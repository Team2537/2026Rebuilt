// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotType;
import frc.robot.generated.TunerConstants;
import frc.robot.util.ElasticNotifications;
import frc.robot.util.LocalADStarAK;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumSet;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    // TunerConstants doesn't include these constants, so they are declared locally
    static final double ODOMETRY_FREQUENCY = new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD()
            ? 250.0
            : 100.0;
    public static final double DRIVE_BASE_RADIUS = Math.max(
            Math.max(
                    Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                    Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
            Math.max(
                    Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                    Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

    // PathPlanner config constants
    private static final double LOOP_PERIOD_SECONDS = 0.02;
    private static final RobotConfig PP_CONFIG = new RobotConfig(
            DriveConstants.ROBOT_MASS_KG,
            DriveConstants.ROBOT_MOI,
            new ModuleConfig(
                    TunerConstants.FrontLeft.WheelRadius,
                    TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                    DriveConstants.WHEEL_COF,
                    DCMotor.getKrakenX60Foc(1)
                            .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                    TunerConstants.FrontLeft.SlipCurrent,
                    1),
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY));

    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;
    private final SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(
            PP_CONFIG,
            DriveConstants.MAX_STEER_VELOCITY_RAD_PER_SEC);
    private final EnumSet<DriveConstants.ConstraintProfile> activeConstraintProfiles =
            EnumSet.noneOf(DriveConstants.ConstraintProfile.class);
    private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
            AlertType.kError);
    private StringLogEntry sysIdStateLogEntry;
    private boolean lastGyroConnected = true;
    private double characterizationVolts = 0.0;
    private SwerveSetpoint previousSetpoint;
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };
    // Pre-allocated arrays reused each odometry cycle to avoid GC pressure
    private final SwerveModulePosition[] odomModulePositions = new SwerveModulePosition[4];
    private final SwerveModulePosition[] odomModuleDeltas = new SwerveModulePosition[4];
    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation,
            lastModulePositions, Pose2d.kZero);
    // Tracks pure odometry (no vision corrections) in sim mode so the vision sim
    // cameras see the true robot position rather than the fused estimate.
    private final SwerveDriveOdometry simGroundTruthOdometry;
    private boolean fieldOriented = true;

    public Drive(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
        modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
        modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
        modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);
        previousSetpoint = createStoppedSetpointFromModuleAngles();
        logConstraintState();

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        double autoRotationKp = RobotType.MODE == RobotType.Mode.SIMULATION ? 7.5 : 5.0;

        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getChassisSpeeds,
                (speeds, feedforwards) -> runVelocity(speeds, feedforwards),
                new PPHolonomicDriveController(
                        new PIDConstants(7.0, 0.0, 0.0),
                        new PIDConstants(autoRotationKp, 0.0, 0.0)),
                PP_CONFIG,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                            "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                });
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                });

        // Sim ground truth odometry (vision-free pose for camera placement)
        simGroundTruthOdometry = RobotType.MODE == RobotType.Mode.SIMULATION
                ? new SwerveDriveOdometry(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero)
                : null;

        // Configure SysId
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        (state) -> {
                            String stateString = state.toString();
                            if (sysIdStateLogEntry == null) {
                                sysIdStateLogEntry =
                                        new StringLogEntry(DataLogManager.getLog(), "sysid-test-state-" + getName());
                            }
                            sysIdStateLogEntry.append(stateString);
                            Logger.recordOutput("Drive/SysIdState", stateString);
                            System.out.println("[Drive SysId] " + stateString);
                        }),
                new SysIdRoutine.Mechanism(
                        (voltage) -> {
                            characterizationVolts = voltage.in(Volts);
                            runCharacterization(characterizationVolts);
                        },
                        (log) -> {
                            double avgPositionMeters = 0.0;
                            double avgVelocityMetersPerSec = 0.0;
                            for (int i = 0; i < 4; i++) {
                                avgPositionMeters += modules[i].getPositionMeters() / 4.0;
                                avgVelocityMetersPerSec += modules[i].getVelocityMetersPerSec() / 4.0;
                            }

                            log.motor("drive")
                                    .voltage(Volts.of(characterizationVolts))
                                    .linearPosition(Meters.of(avgPositionMeters))
                                    .linearVelocity(MetersPerSecond.of(avgVelocityMetersPerSec));
                        },
                        this));
    }

    @Override
    public void periodic() {
        odometryLock.lock();
        try {
            gyroIO.updateInputs(gyroInputs);
            Logger.processInputs("Drive/Gyro", gyroInputs);
            for (var module : modules) {
                module.periodic();
            }
        } finally {
            odometryLock.unlock();
        }

        // Stop moving and log empty setpoints when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
            previousSetpoint = createStoppedSetpointFromModuleAngles();
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveChassisSpeeds/Setpoints", new ChassisSpeeds());
        }

        // Update odometry
        int[] moduleSampleCounts = new int[modules.length];
        int sampleCount = Integer.MAX_VALUE;
        for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
            moduleSampleCounts[moduleIndex] = Math.min(
                    modules[moduleIndex].getOdometryTimestamps().length,
                    modules[moduleIndex].getOdometryPositions().length);
            sampleCount = Math.min(sampleCount, moduleSampleCounts[moduleIndex]);
        }
        int gyroSampleCount = gyroInputs.connected
                ? Math.min(gyroInputs.odometryYawTimestamps.length, gyroInputs.odometryYawPositions.length)
                : 0;
        if (gyroInputs.connected) {
            sampleCount = Math.min(sampleCount, gyroSampleCount);
        }
        if (sampleCount == Integer.MAX_VALUE) {
            sampleCount = 0;
        }
        final int sharedSampleCount = sampleCount;

        Logger.recordOutput("Drive/Odometry/ModuleSampleCounts", moduleSampleCounts);
        Logger.recordOutput("Drive/Odometry/GyroSampleCount", gyroSampleCount);
        Logger.recordOutput("Drive/Odometry/SharedSampleCount", sharedSampleCount);
        Logger.recordOutput(
                "Drive/Odometry/SampleCountMismatch",
                Arrays.stream(moduleSampleCounts).anyMatch(count -> count != sharedSampleCount)
                        || (gyroInputs.connected && gyroSampleCount != sharedSampleCount));
        for (int i = 0; i < sharedSampleCount; i++) {
            // Read wheel positions and deltas from each module (reuse pre-allocated arrays)
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                SwerveModulePosition pos = modules[moduleIndex].getOdometryPositions()[i];
                odomModulePositions[moduleIndex] = pos;
                odomModuleDeltas[moduleIndex] = new SwerveModulePosition(
                        pos.distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        pos.angle);
                lastModulePositions[moduleIndex] = pos;
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(odomModuleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            double sampleTimestamp = getOdometrySampleTimestamp(i);
            poseEstimator.updateWithTime(sampleTimestamp, rawGyroRotation, odomModulePositions);
            if (simGroundTruthOdometry != null) {
                simGroundTruthOdometry.update(rawGyroRotation, odomModulePositions);
            }
        }

        // Update gyro alert and notification
        boolean gyroConnected = gyroInputs.connected;
        gyroDisconnectedAlert.set(!gyroConnected && RobotType.MODE != RobotType.Mode.SIMULATION);
        Logger.recordOutput("RobotState/GyroConnected", gyroConnected);
        if (!gyroConnected && lastGyroConnected && RobotType.MODE != RobotType.Mode.SIMULATION) {
            ElasticNotifications.sendError("Gyro Disconnected", "Using kinematics fallback");
        }
        lastGyroConnected = gyroConnected;
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        applySetpoint(speeds, null, null);
    }

    /**
     * Runs the drive at the desired velocity and logs pathplanner feedforwards.
     *
     * @param speeds Speeds in meters/sec
     * @param feedforwards PathPlanner-generated feedforwards in module order FL,FR,BL,BR
     */
    public void runVelocity(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
        applySetpoint(speeds, null, feedforwards);
    }

    /**
     * Runs the drive at the desired driver-commanded velocity using active teleop
     * constraints.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runDriverVelocity(ChassisSpeeds speeds) {
        applySetpoint(speeds, getResolvedDriverConstraints(), null);
    }

    private void applySetpoint(
            ChassisSpeeds desiredSpeeds,
            PathConstraints constraints,
            DriveFeedforwards pathplannerFeedforwards) {
        SwerveSetpoint generatedSetpoint =
                setpointGenerator.generateSetpoint(previousSetpoint, desiredSpeeds, constraints, LOOP_PERIOD_SECONDS);
        SwerveModuleState[] rawSetpointStates = copyModuleStates(generatedSetpoint.moduleStates());
        SwerveModuleState[] appliedSetpointStates = copyModuleStates(rawSetpointStates);
        DriveFeedforwards selectedFeedforwards =
                pathplannerFeedforwards != null ? pathplannerFeedforwards : generatedSetpoint.feedforwards();
        if (selectedFeedforwards == null) {
            selectedFeedforwards = DriveFeedforwards.zeros(modules.length);
        }

        Logger.recordOutput("SwerveChassisSpeeds/Requested", desiredSpeeds);
        Logger.recordOutput("Drive/SetpointGenerator/UsingExternalFeedforwards", pathplannerFeedforwards != null);
        Logger.recordOutput(
                "Drive/SetpointGenerator/FeedforwardAccelerationsMpsSq",
                selectedFeedforwards.accelerationsMPSSq());
        Logger.recordOutput("Drive/SetpointGenerator/FeedforwardTorqueCurrentsAmps",
                selectedFeedforwards.torqueCurrentsAmps());
        Logger.recordOutput("Drive/SetpointGenerator/FeedforwardLinearForcesN",
                selectedFeedforwards.linearForcesNewtons());

        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(
                    appliedSetpointStates[i],
                    false,
                    getFeedforwardValue(selectedFeedforwards.accelerationsMPSSq(), i),
                    getModuleArbitraryFeedforward(selectedFeedforwards, i));
        }

        ChassisSpeeds appliedSpeeds = kinematics.toChassisSpeeds(appliedSetpointStates);
        previousSetpoint = new SwerveSetpoint(
                appliedSpeeds,
                copyModuleStates(appliedSetpointStates),
                selectedFeedforwards);

        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", appliedSpeeds);
        Logger.recordOutput("SwerveStates/Setpoints", rawSetpointStates);
        Logger.recordOutput("SwerveStates/SetpointsOptimized", appliedSetpointStates);
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement.
     * The modules will
     * return to their normal orientations the next time a nonzero velocity is
     * requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);

        // Immediately apply zero-speed setpoints with the new headings.
        // resetHeadings() keeps these angles as the preferred headings for future zero-speed commands.
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(new ChassisSpeeds());
        SwerveModuleState[] rawSetpointStates = copyModuleStates(setpointStates);
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i], false);
        }
        previousSetpoint = new SwerveSetpoint(
                new ChassisSpeeds(),
                copyModuleStates(setpointStates),
                DriveFeedforwards.zeros(modules.length));

        Logger.recordOutput("SwerveStates/Setpoints", rawSetpointStates);
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", new ChassisSpeeds());
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the
     * modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the
     * modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns module positions for simulation helpers. */
    public SwerveModulePosition[] getModulePositionsForSim() {
        return getModulePositions();
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns measured chassis speeds in robot-relative coordinates. */
    public ChassisSpeeds getMeasuredChassisSpeeds() {
        return getChassisSpeeds();
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /**
     * Returns the average velocity of the modules in rotations/sec (Phoenix native
     * units).
     */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Returns the sim ground truth pose (pure odometry, no vision corrections).
     * Falls back to the fused estimator pose when not in simulation.
     */
    public Pose2d getSimGroundTruthPose() {
        return simGroundTruthOdometry != null ? simGroundTruthOdometry.getPoseMeters() : getPose();
    }

    /**
     * Returns the estimator pose sampled at a historical timestamp.
     * Falls back to current pose if the estimator buffer does not have that sample.
     */
    public Pose2d getPoseAtTimestamp(double timestampSeconds) {
        if (!Double.isFinite(timestampSeconds)) {
            return getPose();
        }
        Optional<Pose2d> sampledPose = poseEstimator.sampleAt(timestampSeconds);
        return sampledPose.orElseGet(this::getPose);
    }

    /** Returns the current robot rotation for field orientation. */
    public Rotation2d getRotation() {
        // On real hardware prefer live gyro yaw; in sim/disconnected, use pose rotation
        return gyroInputs.connected ? gyroInputs.yawPosition : getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        Rotation2d rotation = pose.getRotation();
        odometryLock.lock();
        try {
            rawGyroRotation = rotation;
            gyroInputs.yawPosition = rotation;
            gyroIO.setYaw(rotation);
            poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
            if (simGroundTruthOdometry != null) {
                simGroundTruthOdometry.resetPosition(rawGyroRotation, getModulePositions(), pose);
            }
        } finally {
            odometryLock.unlock();
        }
    }

    private static SwerveModuleState[] copyModuleStates(SwerveModuleState[] states) {
        SwerveModuleState[] copy = new SwerveModuleState[states.length];
        for (int i = 0; i < states.length; i++) {
            copy[i] = new SwerveModuleState(states[i].speedMetersPerSecond, states[i].angle);
        }
        return copy;
    }

    private SwerveSetpoint createStoppedSetpointFromModuleAngles() {
        SwerveModuleState[] holdStates = getModuleStates();
        for (SwerveModuleState state : holdStates) {
            state.speedMetersPerSecond = 0.0;
        }
        return new SwerveSetpoint(
                new ChassisSpeeds(),
                holdStates,
                DriveFeedforwards.zeros(modules.length));
    }

    private PathConstraints getResolvedDriverConstraints() {
        ArrayList<PathConstraints> overlays = new ArrayList<>(activeConstraintProfiles.size());
        for (DriveConstants.ConstraintProfile profile : activeConstraintProfiles) {
            overlays.add(DriveConstants.constraintsForProfile(profile));
        }
        return DriveConstants.mergeConstraints(DriveConstants.DRIVER_DEFAULT_LIMITS, overlays);
    }

    private void logConstraintState() {
        PathConstraints resolved = getResolvedDriverConstraints();
        String[] activeProfiles = activeConstraintProfiles.stream()
                .map(Enum::name)
                .sorted()
                .toArray(String[]::new);
        Logger.recordOutput("Drive/ConstraintProfilesActive", activeProfiles);
        Logger.recordOutput(
                "Drive/ConstraintProfile/SlowMode",
                isConstraintProfileActive(DriveConstants.ConstraintProfile.SLOW_MODE));
        Logger.recordOutput(
                "Drive/ConstraintProfile/ShootingOnMove",
                isConstraintProfileActive(DriveConstants.ConstraintProfile.SHOOTING_ON_MOVE));
        Logger.recordOutput("Drive/ConstraintMaxVelocityMps", resolved.maxVelocityMPS());
        Logger.recordOutput("Drive/ConstraintMaxAccelerationMpsSq", resolved.maxAccelerationMPSSq());
        Logger.recordOutput("Drive/ConstraintMaxAngularVelocityRadPerSec", resolved.maxAngularVelocityRadPerSec());
        Logger.recordOutput(
                "Drive/ConstraintMaxAngularAccelerationRadPerSecSq",
                resolved.maxAngularAccelerationRadPerSecSq());
        boolean slowMode = isConstraintProfileActive(DriveConstants.ConstraintProfile.SLOW_MODE);
        Logger.recordOutput("Drive/SlowMode", slowMode);
        Logger.recordOutput("RobotState/SlowMode", slowMode);
    }

    public void setConstraintProfileActive(DriveConstants.ConstraintProfile profile, boolean enabled) {
        boolean changed = enabled ? activeConstraintProfiles.add(profile) : activeConstraintProfiles.remove(profile);
        if (changed) {
            logConstraintState();
        }
    }

    public boolean isConstraintProfileActive(DriveConstants.ConstraintProfile profile) {
        return activeConstraintProfiles.contains(profile);
    }

    /** Adds a new timestamped vision measurement. */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(
                visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return getResolvedDriverConstraints().maxVelocityMPS();
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return getResolvedDriverConstraints().maxAngularVelocityRadPerSec();
    }

    /** Returns whether drive is field oriented. */
    public boolean isFieldOriented() {
        return fieldOriented;
    }

    /** Sets field oriented mode. */
    public void setFieldOriented(boolean fieldOriented) {
        this.fieldOriented = fieldOriented;
        Logger.recordOutput("Drive/FieldOriented", fieldOriented);
        Logger.recordOutput("RobotState/FieldOriented", fieldOriented);
    }

    /** Toggles field oriented mode. */
    public void toggleFieldOriented() {
        setFieldOriented(!fieldOriented);
    }

    /** Resets odometry and sets current heading/yaw to zero. */
    public void resetOdometryAndHeadingToZero() {
        Pose2d current = getPose();
        setPose(new Pose2d(current.getTranslation(), Rotation2d.kZero));
        Logger.recordOutput("Drive/HeadingReset", true);
    }

    private static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return MODULE_TRANSLATIONS;
    }

    /** Returns a command that toggles slow mode. */
    public Command toggleSlowMode() {
        return Commands.runOnce(() -> {
            boolean enableSlowMode = !isConstraintProfileActive(DriveConstants.ConstraintProfile.SLOW_MODE);
            setConstraintProfileActive(DriveConstants.ConstraintProfile.SLOW_MODE, enableSlowMode);
        }, this);
    }

    private double getOdometrySampleTimestamp(int sampleIndex) {
        double timestampSum = 0.0;
        int timestampCount = 0;

        for (Module module : modules) {
            double[] timestamps = module.getOdometryTimestamps();
            if (sampleIndex < timestamps.length && Double.isFinite(timestamps[sampleIndex])) {
                timestampSum += timestamps[sampleIndex];
                timestampCount++;
            }
        }
        if (gyroInputs.connected
                && sampleIndex < gyroInputs.odometryYawTimestamps.length
                && Double.isFinite(gyroInputs.odometryYawTimestamps[sampleIndex])) {
            timestampSum += gyroInputs.odometryYawTimestamps[sampleIndex];
            timestampCount++;
        }

        return timestampCount > 0 ? timestampSum / timestampCount : 0.0;
    }

    private double getModuleArbitraryFeedforward(DriveFeedforwards feedforwards, int moduleIndex) {
        if (feedforwards == null) {
            return 0.0;
        }
        return switch (modules[moduleIndex].getDriveClosedLoopOutput()) {
            case Voltage -> 0.0;
            case TorqueCurrentFOC -> getFeedforwardValue(feedforwards.torqueCurrentsAmps(), moduleIndex);
        };
    }

    private static double getFeedforwardValue(double[] values, int index) {
        return values != null && index < values.length ? values[index] : 0.0;
    }
}
