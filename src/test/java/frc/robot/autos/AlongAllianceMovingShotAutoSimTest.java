package frc.robot.autos;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.coordination.shooting.ShootCoordinator;
import frc.robot.generated.TunerConstants;
import frc.robot.sim.FuelSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.LaunchCalculator;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.transfer.Transfer;
import frc.robot.subsystems.transfer.TransferIOSim;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.stream.Collectors;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

class AlongAllianceMovingShotAutoSimTest {
    private static final String DEFAULT_AUTO_NAME = "Along Alliance Moving Shot";
    private static final double LOOP_PERIOD_SEC = 0.02;
    private static final double MAX_SIM_TIME_SEC = 45.0;
    private static final double DEFAULT_POST_AUTO_OBSERVATION_SEC = 3.0;
    private static final double MOVING_SPEED_MIN_MPS = 0.25;
    private static final DateTimeFormatter SUMMARY_TIMESTAMP_FORMAT =
            DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss");

    private DiagnosticsConfig diagnosticsConfig;
    private Path diagnosticsOutputDir;
    private String autoName;

    @BeforeEach
    void setUp() throws IOException {
        HAL.initialize(500, 0);
        AutoBuilder.resetForTesting();
        SimHooks.pauseTiming();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setTest(false);
        DriverStationSim.setEStop(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        CommandScheduler.getInstance().cancelAll();

        diagnosticsConfig = DiagnosticsConfig.fromSystemProperties();
        diagnosticsOutputDir = diagnosticsConfig.outputDir();
        autoName = configuredAutoName();
        Files.createDirectories(diagnosticsOutputDir);

        Logger.end();
        Logger.AdvancedHooks.disableRobotBaseCheck();
        Logger.disableConsoleCapture();
        Logger.recordMetadata("TestName", getClass().getSimpleName());
        Logger.recordMetadata("AutoName", autoName);
        Logger.recordMetadata("OutputDir", diagnosticsOutputDir.toString());
        Logger.addDataReceiver(new WPILOGWriter(diagnosticsOutputDir.toString()));
        Logger.start();
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        AutoBuilder.resetForTesting();
        Logger.end();
        SimHooks.resumeTiming();
    }

    @Test
    void alongAllianceMovingShotAutoProducesAccurateAimDiagnostics() throws IOException {
        GyroIOSim gyro = new GyroIOSim(Drive.getModuleTranslations());
        Drive drive = new Drive(
                gyro,
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        gyro.setModulePositionsSupplier(drive::getModulePositionsForSim);

        Shooter shooter = new Shooter(new ShooterIOSim());
        Transfer transfer = new Transfer(new TransferIOSim());
        Intake intake = new Intake(new IntakeIOSim());
        ShootCoordinator shootCoordinator = new ShootCoordinator(shooter, transfer);
        FuelSim fuelSim = new FuelSim();

        frc.robot.RobotState.initialize(drive);
        AutoNamedCommands.registerAll(drive, shooter, transfer, intake, shootCoordinator);

        assertTrue(
                AutoRoutines.create(drive).stream()
                        .map(AutoRoutines.AutoRoutine::name)
                        .collect(Collectors.toSet())
                        .contains("pp/" + autoName),
                "Auto selector routines did not include PathPlanner auto: " + autoName);

        Command auto = new PathPlannerAuto(autoName).withName("TestAuto_" + autoName);
        CommandScheduler.getInstance().schedule(auto);

        AimDiagnosticsAccumulator diagnostics =
                new AimDiagnosticsAccumulator(diagnosticsConfig.idealDescentMissDistanceMeters());
        Pose2d previousPose = drive.getPose();

        boolean finished = false;
        double elapsedSec = 0.0;
        double autoFinishedAtSec = Double.NaN;
        int maxIterations = (int) Math.ceil(MAX_SIM_TIME_SEC / LOOP_PERIOD_SEC);
        for (int i = 0; i < maxIterations; i++) {
            SimHooks.stepTiming(LOOP_PERIOD_SEC);
            DriverStationSim.notifyNewData();
            runLoggedSchedulerCycle();

            double simTimeSec = (i + 1) * LOOP_PERIOD_SEC;
            Pose2d currentPose = drive.getPose();
            fuelSim.update(
                    currentPose,
                    shooter.getMeasuredAverageShooterRpm(),
                    shooter.getTargetHoodAngleRad(),
                    shooter.isKickerActive(),
                    LOOP_PERIOD_SEC);
            diagnostics.record(simTimeSec, previousPose, currentPose, drive, shooter, fuelSim);
            previousPose = currentPose;

            if (!finished && !CommandScheduler.getInstance().isScheduled(auto)) {
                finished = true;
                autoFinishedAtSec = simTimeSec;
            }
            if (finished
                    && Double.isFinite(autoFinishedAtSec)
                    && simTimeSec - autoFinishedAtSec >= diagnosticsConfig.postAutoObservationSec()) {
                elapsedSec = simTimeSec;
                break;
            }
        }

        if (!finished) {
            elapsedSec = maxIterations * LOOP_PERIOD_SEC;
        }
        DiagnosticsSummary summary = diagnostics.finish(autoName, finished, elapsedSec);
        Path summaryFile = writeSummary(summary);
        logSummary(summary, summaryFile);

        assertTrue(summary.finished(), "PathPlanner auto did not finish in simulation.");
        assertTrue(
                summary.distanceTraveledMeters() > diagnosticsConfig.minDistanceTraveledMeters(),
                "Auto moved too little distance. distanceTraveledMeters=" + summary.distanceTraveledMeters());
        assertTrue(
                summary.movingFeedSamples() >= diagnosticsConfig.minMovingFeedSamples(),
                "Insufficient on-the-move feed samples. movingFeedSamples=" + summary.movingFeedSamples());
        assertTrue(
                summary.movingFeedFraction() >= diagnosticsConfig.minMovingFeedFraction(),
                "Feed did not stay on while moving enough. movingFeedFraction=" + summary.movingFeedFraction());
        assertTrue(
                summary.p90MovingAimErrorDeg() <= diagnosticsConfig.maxP90MovingAimErrorDeg(),
                String.format(
                        Locale.US,
                        "P90 aim error too high during moving feed. p90=%.3f deg maxAllowed=%.3f deg",
                        summary.p90MovingAimErrorDeg(),
                        diagnosticsConfig.maxP90MovingAimErrorDeg()));
        assertTrue(
                summary.maxMovingAimErrorDeg() <= diagnosticsConfig.maxMovingAimErrorDeg(),
                String.format(
                        Locale.US,
                        "Max aim error too high during moving feed. max=%.3f deg maxAllowed=%.3f deg",
                        summary.maxMovingAimErrorDeg(),
                        diagnosticsConfig.maxMovingAimErrorDeg()));
        assertTrue(
                summary.descentCrossingSamples() >= diagnosticsConfig.minDescentCrossingSamples(),
                "Too few descent crossing samples at z=1.83m. descentCrossingSamples="
                        + summary.descentCrossingSamples());
        assertTrue(
                summary.p90DescentMissDistanceMeters()
                        <= diagnosticsConfig.maxP90DescentMissDistanceMeters(),
                String.format(
                        Locale.US,
                        "P90 descent miss distance too high. p90=%.4f m (%.2f in) maxAllowed=%.4f m (%.2f in)",
                        summary.p90DescentMissDistanceMeters(),
                        summary.p90DescentMissDistanceInches(),
                        diagnosticsConfig.maxP90DescentMissDistanceMeters(),
                        diagnosticsConfig.maxP90DescentMissDistanceMeters() * 39.37007874));
        assertTrue(
                summary.maxDescentMissDistanceMeters()
                        <= diagnosticsConfig.maxDescentMissDistanceMeters(),
                String.format(
                        Locale.US,
                        "Max descent miss distance too high. max=%.4f m (%.2f in) maxAllowed=%.4f m (%.2f in)",
                        summary.maxDescentMissDistanceMeters(),
                        summary.maxDescentMissDistanceInches(),
                        diagnosticsConfig.maxDescentMissDistanceMeters(),
                        diagnosticsConfig.maxDescentMissDistanceMeters() * 39.37007874));
        assertTrue(
                summary.descentWithinIdealFraction()
                        >= diagnosticsConfig.minDescentWithinIdealFraction(),
                String.format(
                        Locale.US,
                        "Descent ideal-hit fraction too low. actual=%.5f minRequired=%.5f",
                        summary.descentWithinIdealFraction(),
                        diagnosticsConfig.minDescentWithinIdealFraction()));
    }

    private Path writeSummary(DiagnosticsSummary summary) throws IOException {
        String timestamp = SUMMARY_TIMESTAMP_FORMAT.format(LocalDateTime.now());
        String autoSlug = sanitizeForFilename(summary.autoName());
        Path summaryFile =
                diagnosticsOutputDir.resolve("auto_aim_summary_" + autoSlug + "_" + timestamp + ".txt");
        Files.writeString(summaryFile, summary.toMultilineString(diagnosticsConfig));
        Logger.recordOutput("AutoAimDiagnostics/SummaryFile", summaryFile.toString());
        return summaryFile;
    }

    private static void logSummary(DiagnosticsSummary summary, Path summaryFile) {
        Logger.recordOutput("AutoAimDiagnostics/Summary/Finished", summary.finished());
        Logger.recordOutput("AutoAimDiagnostics/Summary/ElapsedSec", summary.elapsedSec());
        Logger.recordOutput(
                "AutoAimDiagnostics/Summary/DistanceTraveledMeters",
                summary.distanceTraveledMeters());
        Logger.recordOutput("AutoAimDiagnostics/Summary/FeedSamples", summary.feedSamples());
        Logger.recordOutput("AutoAimDiagnostics/Summary/MovingFeedSamples", summary.movingFeedSamples());
        Logger.recordOutput("AutoAimDiagnostics/Summary/MovingFeedFraction", summary.movingFeedFraction());
        Logger.recordOutput("AutoAimDiagnostics/Summary/P50MovingAimErrorDeg", summary.p50MovingAimErrorDeg());
        Logger.recordOutput("AutoAimDiagnostics/Summary/P90MovingAimErrorDeg", summary.p90MovingAimErrorDeg());
        Logger.recordOutput("AutoAimDiagnostics/Summary/MaxMovingAimErrorDeg", summary.maxMovingAimErrorDeg());
        Logger.recordOutput("AutoAimDiagnostics/Summary/MeanMovingAimErrorDeg", summary.meanMovingAimErrorDeg());
        Logger.recordOutput("AutoAimDiagnostics/Summary/MeanMovingFeedTargetRpm", summary.meanMovingFeedTargetRpm());
        Logger.recordOutput(
                "AutoAimDiagnostics/Summary/MeanMovingFeedMeasuredRpm",
                summary.meanMovingFeedMeasuredRpm());
        Logger.recordOutput("AutoAimDiagnostics/Summary/MeanMovingFeedRpmError", summary.meanMovingFeedRpmError());
        Logger.recordOutput(
                "AutoAimDiagnostics/Summary/P90AbsMovingFeedRpmError",
                summary.p90AbsMovingFeedRpmError());
        Logger.recordOutput(
                "AutoAimDiagnostics/Summary/MeanMovingFeedFieldVelocityEstimateErrorMps",
                summary.meanMovingFeedFieldVelocityEstimateErrorMps());
        Logger.recordOutput(
                "AutoAimDiagnostics/Summary/P90MovingFeedFieldVelocityEstimateErrorMps",
                summary.p90MovingFeedFieldVelocityEstimateErrorMps());
        Logger.recordOutput(
                "AutoAimDiagnostics/Summary/MeanMovingFeedOmegaEstimateErrorRadPerSec",
                summary.meanMovingFeedOmegaEstimateErrorRadPerSec());
        Logger.recordOutput(
                "AutoAimDiagnostics/Summary/P90MovingFeedOmegaEstimateErrorRadPerSec",
                summary.p90MovingFeedOmegaEstimateErrorRadPerSec());
        Logger.recordOutput(
                "AutoAimDiagnostics/Summary/DescentCrossingSamples",
                summary.descentCrossingSamples());
        Logger.recordOutput(
                "AutoAimDiagnostics/Summary/DescentWithinIdealSamples",
                summary.descentWithinIdealSamples());
        Logger.recordOutput(
                "AutoAimDiagnostics/Summary/DescentWithinIdealFraction",
                summary.descentWithinIdealFraction());
        Logger.recordOutput(
                "AutoAimDiagnostics/Summary/P50DescentMissDistanceMeters",
                summary.p50DescentMissDistanceMeters());
        Logger.recordOutput(
                "AutoAimDiagnostics/Summary/P90DescentMissDistanceMeters",
                summary.p90DescentMissDistanceMeters());
        Logger.recordOutput(
                "AutoAimDiagnostics/Summary/MaxDescentMissDistanceMeters",
                summary.maxDescentMissDistanceMeters());
        Logger.recordOutput(
                "AutoAimDiagnostics/Summary/MeanDescentMissDistanceMeters",
                summary.meanDescentMissDistanceMeters());
        Logger.recordOutput(
                "AutoAimDiagnostics/Summary/MeanSignedDescentMissDistanceMeters",
                summary.meanSignedDescentMissDistanceMeters());
        Logger.recordOutput(
                "AutoAimDiagnostics/Summary/MeanAlongVelocityMissDistanceMeters",
                summary.meanAlongVelocityMissDistanceMeters());
        Logger.recordOutput(
                "AutoAimDiagnostics/Summary/MeanAbsAlongVelocityMissDistanceMeters",
                summary.meanAbsAlongVelocityMissDistanceMeters());
        Logger.recordOutput(
                "AutoAimDiagnostics/Summary/P90AbsAlongVelocityMissDistanceMeters",
                summary.p90AbsAlongVelocityMissDistanceMeters());
        Logger.recordOutput(
                "AutoAimDiagnostics/Summary/MeanAbsCrossVelocityMissDistanceMeters",
                summary.meanAbsCrossVelocityMissDistanceMeters());
        Logger.recordOutput(
                "AutoAimDiagnostics/Summary/P90AbsCrossVelocityMissDistanceMeters",
                summary.p90AbsCrossVelocityMissDistanceMeters());
        Logger.recordOutput(
                "AutoAimDiagnostics/Summary/MeanAbsVelocityToTargetAngleDeg",
                summary.meanAbsVelocityToTargetAngleDeg());
        Logger.recordOutput(
                "AutoAimDiagnostics/Summary/P90AbsVelocityToTargetAngleDeg",
                summary.p90AbsVelocityToTargetAngleDeg());
        System.out.println(summary.toSingleLineSummary(summaryFile));
    }

    private static void runLoggedSchedulerCycle() {
        long periodicBeforeStartUs = RobotController.getFPGATime();
        Logger.AdvancedHooks.invokePeriodicBeforeUser();
        long periodicBeforeEndUs = RobotController.getFPGATime();

        CommandScheduler.getInstance().run();

        long userCodeEndUs = RobotController.getFPGATime();
        Logger.AdvancedHooks.invokePeriodicAfterUser(
                userCodeEndUs - periodicBeforeEndUs,
                periodicBeforeEndUs - periodicBeforeStartUs);
    }

    private static String getConfiguredValue(String systemPropertyKey, String... environmentKeys) {
        String value = System.getProperty(systemPropertyKey);
        if (value != null && !value.isBlank()) {
            return value;
        }
        for (String environmentKey : environmentKeys) {
            value = System.getenv(environmentKey);
            if (value != null && !value.isBlank()) {
                return value;
            }
        }
        return null;
    }

    private static String configuredAutoName() {
        String configured = getConfiguredValue(
                "autoAim.diag.autoName",
                "AUTO_AIM_DIAG_AUTO_NAME",
                "AUTO_AIM_AUTO_NAME");
        return configured == null || configured.isBlank() ? DEFAULT_AUTO_NAME : configured;
    }

    private static String sanitizeForFilename(String value) {
        return value.toLowerCase(Locale.ROOT).replaceAll("[^a-z0-9]+", "_").replaceAll("^_|_$", "");
    }

    private static double parseDoubleProperty(
            String key,
            double defaultValue,
            String... environmentKeys) {
        String value = getConfiguredValue(key, environmentKeys);
        if (value == null || value.isBlank()) {
            return defaultValue;
        }
        try {
            return Double.parseDouble(value);
        } catch (NumberFormatException exception) {
            throw new IllegalArgumentException(
                    "Expected a numeric system property for " + key + ", got '" + value + "'.",
                    exception);
        }
    }

    private static int parseIntProperty(
            String key,
            int defaultValue,
            String... environmentKeys) {
        String value = getConfiguredValue(key, environmentKeys);
        if (value == null || value.isBlank()) {
            return defaultValue;
        }
        try {
            return Integer.parseInt(value);
        } catch (NumberFormatException exception) {
            throw new IllegalArgumentException(
                    "Expected an integer system property for " + key + ", got '" + value + "'.",
                    exception);
        }
    }

    private record DiagnosticsConfig(
            Path outputDir,
            double minDistanceTraveledMeters,
            int minMovingFeedSamples,
            double minMovingFeedFraction,
            double maxP90MovingAimErrorDeg,
            double maxMovingAimErrorDeg,
            int minDescentCrossingSamples,
            double idealDescentMissDistanceMeters,
            double maxP90DescentMissDistanceMeters,
            double maxDescentMissDistanceMeters,
            double minDescentWithinIdealFraction,
            double postAutoObservationSec) {
        private static DiagnosticsConfig fromSystemProperties() {
            String outputDirValue = getConfiguredValue(
                    "autoAim.diag.outDir",
                    "AUTO_AIM_DIAG_OUT_DIR",
                    "AUTO_AIM_OUT_DIR");
            Path outputDir = Path.of(outputDirValue == null || outputDirValue.isBlank()
                    ? "build/auto-aim-diagnostics"
                    : outputDirValue);
            return new DiagnosticsConfig(
                    outputDir.toAbsolutePath(),
                    parseDoubleProperty(
                            "autoAim.diag.minDistanceMeters",
                            4.0,
                            "AUTO_AIM_DIAG_MIN_DISTANCE_METERS",
                            "AUTO_AIM_MIN_DISTANCE_M"),
                    parseIntProperty(
                            "autoAim.diag.minMovingFeedSamples",
                            10,
                            "AUTO_AIM_DIAG_MIN_MOVING_FEED_SAMPLES",
                            "AUTO_AIM_MIN_MOVING_FEED_SAMPLES"),
                    parseDoubleProperty(
                            "autoAim.diag.minMovingFeedFraction",
                            0.65,
                            "AUTO_AIM_DIAG_MIN_MOVING_FEED_FRACTION",
                            "AUTO_AIM_MIN_MOVING_FEED_FRACTION"),
                    parseDoubleProperty(
                            "autoAim.diag.assertP90Deg",
                            11.0,
                            "AUTO_AIM_DIAG_ASSERT_P90_DEG",
                            "AUTO_AIM_ASSERT_P90_DEG"),
                    parseDoubleProperty(
                            "autoAim.diag.assertMaxDeg",
                            18.0,
                            "AUTO_AIM_DIAG_ASSERT_MAX_DEG",
                            "AUTO_AIM_ASSERT_MAX_DEG"),
                    parseIntProperty(
                            "autoAim.diag.minDescentCrossingSamples",
                            2,
                            "AUTO_AIM_DIAG_MIN_DESCENT_SAMPLES",
                            "AUTO_AIM_MIN_DESCENT_SAMPLES"),
                    parseDoubleProperty(
                            "autoAim.diag.idealDescentMissMeters",
                            FuelSim.IDEAL_DESCENT_MISS_DISTANCE_METERS,
                            "AUTO_AIM_DIAG_IDEAL_DESCENT_MISS_METERS"),
                    parseDoubleProperty(
                            "autoAim.diag.assertDescentP90Meters",
                            0.29,
                            "AUTO_AIM_DIAG_ASSERT_DESCENT_P90_METERS"),
                    parseDoubleProperty(
                            "autoAim.diag.assertDescentMaxMeters",
                            0.32,
                            "AUTO_AIM_DIAG_ASSERT_DESCENT_MAX_METERS"),
                    parseDoubleProperty(
                            "autoAim.diag.assertMinDescentWithinIdealFraction",
                            0.0,
                            "AUTO_AIM_DIAG_ASSERT_MIN_DESCENT_WITHIN_IDEAL_FRACTION"),
                    parseDoubleProperty(
                            "autoAim.diag.postAutoObservationSec",
                            DEFAULT_POST_AUTO_OBSERVATION_SEC,
                            "AUTO_AIM_DIAG_POST_AUTO_OBSERVATION_SEC"));
        }
    }

    private static final class AimDiagnosticsAccumulator {
        private final List<Double> movingAimErrorsDeg = new ArrayList<>();
        private final List<Double> descentMissDistancesMeters = new ArrayList<>();
        private final List<Double> signedDescentMissDistancesMeters = new ArrayList<>();
        private final List<Double> descentAlongVelocityMissMeters = new ArrayList<>();
        private final List<Double> descentCrossVelocityMissMeters = new ArrayList<>();
        private final List<Double> descentVelocityToTargetAngleDeg = new ArrayList<>();
        private final List<Double> movingFeedTargetRpm = new ArrayList<>();
        private final List<Double> movingFeedMeasuredRpm = new ArrayList<>();
        private final List<Double> movingFeedRpmError = new ArrayList<>();
        private final List<Double> movingFeedFieldVelocityEstimateErrorMps = new ArrayList<>();
        private final List<Double> movingFeedOmegaEstimateErrorRadPerSec = new ArrayList<>();
        private final List<String> descentSampleDetails = new ArrayList<>();
        private final double idealDescentMissDistanceMeters;
        private double distanceTraveledMeters = 0.0;
        private int feedSamples = 0;
        private int movingFeedSamples = 0;
        private int descentWithinIdealSamples = 0;

        private AimDiagnosticsAccumulator(double idealDescentMissDistanceMeters) {
            this.idealDescentMissDistanceMeters = idealDescentMissDistanceMeters;
        }

        private void record(
                double simTimeSec,
                Pose2d previousPose,
                Pose2d currentPose,
                Drive drive,
                Shooter shooter,
                FuelSim fuelSim) {
            distanceTraveledMeters +=
                    currentPose.getTranslation().getDistance(previousPose.getTranslation());

            ChassisSpeeds speeds = drive.getMeasuredChassisSpeeds();
            double translationalSpeedMps =
                    Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
            boolean moving = translationalSpeedMps >= MOVING_SPEED_MIN_MPS;
            boolean kickerActive = shooter.isKickerActive();
            ChassisSpeeds fieldRelativeSpeeds =
                    ChassisSpeeds.fromRobotRelativeSpeeds(speeds, currentPose.getRotation());
            double estimatedFieldVxMps =
                    (currentPose.getX() - previousPose.getX()) / LOOP_PERIOD_SEC;
            double estimatedFieldVyMps =
                    (currentPose.getY() - previousPose.getY()) / LOOP_PERIOD_SEC;
            double fieldVelocityEstimateErrorMps = Math.hypot(
                    fieldRelativeSpeeds.vxMetersPerSecond - estimatedFieldVxMps,
                    fieldRelativeSpeeds.vyMetersPerSecond - estimatedFieldVyMps);
            double estimatedOmegaRadPerSec = MathUtil.angleModulus(
                    currentPose.getRotation().minus(previousPose.getRotation()).getRadians()) / LOOP_PERIOD_SEC;
            double omegaEstimateErrorRadPerSec =
                    Math.abs(speeds.omegaRadiansPerSecond - estimatedOmegaRadPerSec);

            LaunchCalculator.MotionCompensation compensation =
                    shooter.getMotionCompensationToHub(currentPose, speeds);
            Rotation2d compensatedHeading = compensation.compensatedHeading();
            double absAimErrorDeg = calculateAimErrorDeg(drive.getRotation(), compensatedHeading);
            boolean finiteAimError = Double.isFinite(absAimErrorDeg);

            if (kickerActive) {
                feedSamples++;
                if (moving) {
                    movingFeedSamples++;
                    double targetRpm = shooter.getTargetAverageShooterRpm();
                    double measuredRpm = shooter.getMeasuredAverageShooterRpm();
                    movingFeedTargetRpm.add(targetRpm);
                    movingFeedMeasuredRpm.add(measuredRpm);
                    movingFeedRpmError.add(targetRpm - measuredRpm);
                    movingFeedFieldVelocityEstimateErrorMps.add(fieldVelocityEstimateErrorMps);
                    movingFeedOmegaEstimateErrorRadPerSec.add(omegaEstimateErrorRadPerSec);
                    if (finiteAimError) {
                        movingAimErrorsDeg.add(absAimErrorDeg);
                    }
                }
            }

            Logger.recordOutput("AutoAimDiagnostics/SimTimeSec", simTimeSec);
            Logger.recordOutput("AutoAimDiagnostics/TranslationalSpeedMps", translationalSpeedMps);
            Logger.recordOutput("AutoAimDiagnostics/KickerActive", kickerActive);
            Logger.recordOutput("AutoAimDiagnostics/Moving", moving);
            Logger.recordOutput("AutoAimDiagnostics/CompensatedDistanceMeters",
                    compensation.compensatedDistanceMeters());
            Logger.recordOutput("AutoAimDiagnostics/CompensatedHeadingDeg",
                    compensatedHeading != null ? compensatedHeading.getDegrees() : Double.NaN);
            Logger.recordOutput("AutoAimDiagnostics/MovingAbsAimErrorDeg",
                    moving ? absAimErrorDeg : Double.NaN);
            Logger.recordOutput(
                    "AutoAimDiagnostics/FieldVelocityEstimateErrorMps",
                    fieldVelocityEstimateErrorMps);
            Logger.recordOutput(
                    "AutoAimDiagnostics/OmegaEstimateErrorRadPerSec",
                    omegaEstimateErrorRadPerSec);

            List<FuelSim.DescentCrossingSample> descentSamples = fuelSim.drainDescentCrossingSamples();
            Logger.recordOutput("AutoAimDiagnostics/DescentCrossing/NewSamples", descentSamples.size());
            for (FuelSim.DescentCrossingSample descentSample : descentSamples) {
                double missDistanceMeters = descentSample.missDistanceMeters();
                double signedMissDistanceMeters = descentSample.signedMissDistanceMeters();
                double alongVelocityMissMeters = descentSample.alongVelocityMissMeters();
                double crossVelocityMissMeters = descentSample.crossVelocityMissMeters();
                double velocityToTargetAngleDeg = descentSample.velocityToTargetAngleDeg();
                descentMissDistancesMeters.add(missDistanceMeters);
                signedDescentMissDistancesMeters.add(signedMissDistanceMeters);
                descentAlongVelocityMissMeters.add(alongVelocityMissMeters);
                descentCrossVelocityMissMeters.add(crossVelocityMissMeters);
                descentVelocityToTargetAngleDeg.add(velocityToTargetAngleDeg);
                if (missDistanceMeters <= idealDescentMissDistanceMeters) {
                    descentWithinIdealSamples++;
                }
                Logger.recordOutput("AutoAimDiagnostics/DescentCrossing/LastTimestampSec",
                        descentSample.timestampSec());
                Logger.recordOutput("AutoAimDiagnostics/DescentCrossing/LastX", descentSample.xMeters());
                Logger.recordOutput("AutoAimDiagnostics/DescentCrossing/LastY", descentSample.yMeters());
                Logger.recordOutput("AutoAimDiagnostics/DescentCrossing/LastMissDistanceMeters", missDistanceMeters);
                Logger.recordOutput("AutoAimDiagnostics/DescentCrossing/LastMissDistanceInches",
                        missDistanceMeters * 39.37007874);
                Logger.recordOutput(
                        "AutoAimDiagnostics/DescentCrossing/LastSignedMissDistanceMeters",
                        signedMissDistanceMeters);
                Logger.recordOutput(
                        "AutoAimDiagnostics/DescentCrossing/LastAlongVelocityMissMeters",
                        alongVelocityMissMeters);
                Logger.recordOutput(
                        "AutoAimDiagnostics/DescentCrossing/LastCrossVelocityMissMeters",
                        crossVelocityMissMeters);
                Logger.recordOutput(
                        "AutoAimDiagnostics/DescentCrossing/LastVelocityToTargetAngleDeg",
                        velocityToTargetAngleDeg);
                descentSampleDetails.add(String.format(
                        Locale.US,
                        "%.3f,%.3f,%.3f,%.3f,%.3f",
                        descentSample.timestampSec(),
                        missDistanceMeters * 39.37007874,
                        alongVelocityMissMeters * 39.37007874,
                        crossVelocityMissMeters * 39.37007874,
                        velocityToTargetAngleDeg));
            }
        }

        private DiagnosticsSummary finish(String autoName, boolean finished, double elapsedSec) {
            double p50 = percentile(movingAimErrorsDeg, 0.50);
            double p90 = percentile(movingAimErrorsDeg, 0.90);
            double max = movingAimErrorsDeg.stream().mapToDouble(Double::doubleValue).max().orElse(Double.NaN);
            double mean = movingAimErrorsDeg.stream().mapToDouble(Double::doubleValue).average().orElse(Double.NaN);
            double p50DescentMiss = percentile(descentMissDistancesMeters, 0.50);
            double p90DescentMiss = percentile(descentMissDistancesMeters, 0.90);
            double maxDescentMiss = descentMissDistancesMeters.stream()
                    .mapToDouble(Double::doubleValue)
                    .max()
                    .orElse(Double.NaN);
            double meanDescentMiss = descentMissDistancesMeters.stream()
                    .mapToDouble(Double::doubleValue)
                    .average()
                    .orElse(Double.NaN);
            double meanSignedDescentMiss = signedDescentMissDistancesMeters.stream()
                    .mapToDouble(Double::doubleValue)
                    .average()
                    .orElse(Double.NaN);
            double meanAlongVelocityMiss = descentAlongVelocityMissMeters.stream()
                    .mapToDouble(Double::doubleValue)
                    .average()
                    .orElse(Double.NaN);
            double meanAbsAlongVelocityMiss = descentAlongVelocityMissMeters.stream()
                    .mapToDouble(Math::abs)
                    .average()
                    .orElse(Double.NaN);
            double p90AbsAlongVelocityMiss = percentile(
                    descentAlongVelocityMissMeters.stream().map(Math::abs).toList(),
                    0.90);
            double meanAbsCrossVelocityMiss = descentCrossVelocityMissMeters.stream()
                    .mapToDouble(Math::abs)
                    .average()
                    .orElse(Double.NaN);
            double p90AbsCrossVelocityMiss = percentile(
                    descentCrossVelocityMissMeters.stream().map(Math::abs).toList(),
                    0.90);
            double meanAbsVelocityToTargetAngleDeg = descentVelocityToTargetAngleDeg.stream()
                    .mapToDouble(Math::abs)
                    .average()
                    .orElse(Double.NaN);
            double p90AbsVelocityToTargetAngleDeg = percentile(
                    descentVelocityToTargetAngleDeg.stream().map(Math::abs).toList(),
                    0.90);
            double meanMovingFeedTargetRpm = movingFeedTargetRpm.stream()
                    .mapToDouble(Double::doubleValue)
                    .average()
                    .orElse(Double.NaN);
            double meanMovingFeedMeasuredRpm = movingFeedMeasuredRpm.stream()
                    .mapToDouble(Double::doubleValue)
                    .average()
                    .orElse(Double.NaN);
            double meanMovingFeedRpmError = movingFeedRpmError.stream()
                    .mapToDouble(Double::doubleValue)
                    .average()
                    .orElse(Double.NaN);
            double p90AbsMovingFeedRpmError = percentile(
                    movingFeedRpmError.stream().map(Math::abs).toList(),
                    0.90);
            double meanMovingFeedFieldVelocityEstimateErrorMps =
                    movingFeedFieldVelocityEstimateErrorMps.stream()
                            .mapToDouble(Double::doubleValue)
                            .average()
                            .orElse(Double.NaN);
            double p90MovingFeedFieldVelocityEstimateErrorMps = percentile(
                    movingFeedFieldVelocityEstimateErrorMps,
                    0.90);
            double meanMovingFeedOmegaEstimateErrorRadPerSec =
                    movingFeedOmegaEstimateErrorRadPerSec.stream()
                            .mapToDouble(Double::doubleValue)
                            .average()
                            .orElse(Double.NaN);
            double p90MovingFeedOmegaEstimateErrorRadPerSec = percentile(
                    movingFeedOmegaEstimateErrorRadPerSec,
                    0.90);
            double movingFeedFraction = feedSamples > 0
                    ? (double) movingFeedSamples / feedSamples
                    : 0.0;
            double descentWithinIdealFraction = descentMissDistancesMeters.isEmpty()
                    ? 0.0
                    : (double) descentWithinIdealSamples / descentMissDistancesMeters.size();
            return new DiagnosticsSummary(
                    autoName,
                    finished,
                    elapsedSec,
                    distanceTraveledMeters,
                    feedSamples,
                    movingFeedSamples,
                    movingFeedFraction,
                    p50,
                    p90,
                    max,
                    mean,
                    descentMissDistancesMeters.size(),
                    descentWithinIdealSamples,
                    descentWithinIdealFraction,
                    p50DescentMiss,
                    p90DescentMiss,
                    maxDescentMiss,
                    meanDescentMiss,
                    meanSignedDescentMiss,
                    meanAlongVelocityMiss,
                    meanAbsAlongVelocityMiss,
                    p90AbsAlongVelocityMiss,
                    meanAbsCrossVelocityMiss,
                    p90AbsCrossVelocityMiss,
                    meanAbsVelocityToTargetAngleDeg,
                    p90AbsVelocityToTargetAngleDeg,
                    meanMovingFeedTargetRpm,
                    meanMovingFeedMeasuredRpm,
                    meanMovingFeedRpmError,
                    p90AbsMovingFeedRpmError,
                    meanMovingFeedFieldVelocityEstimateErrorMps,
                    p90MovingFeedFieldVelocityEstimateErrorMps,
                    meanMovingFeedOmegaEstimateErrorRadPerSec,
                    p90MovingFeedOmegaEstimateErrorRadPerSec,
                    String.join(";", descentSampleDetails));
        }

        private static double calculateAimErrorDeg(
                Rotation2d robotHeading,
                Rotation2d compensatedHeading) {
            if (robotHeading == null || compensatedHeading == null) {
                return Double.NaN;
            }
            Rotation2d desiredRobotHeading = compensatedHeading.plus(Rotation2d.kPi);
            double errorRad = MathUtil.angleModulus(
                    desiredRobotHeading.minus(robotHeading).getRadians());
            return Math.abs(Math.toDegrees(errorRad));
        }

        private static double percentile(List<Double> samples, double percentile) {
            if (samples.isEmpty()) {
                return Double.NaN;
            }
            List<Double> sorted = new ArrayList<>(samples);
            sorted.sort(Comparator.naturalOrder());
            double scaledIndex = percentile * (sorted.size() - 1);
            int lowerIndex = (int) Math.floor(scaledIndex);
            int upperIndex = (int) Math.ceil(scaledIndex);
            if (lowerIndex == upperIndex) {
                return sorted.get(lowerIndex);
            }
            double lowerValue = sorted.get(lowerIndex);
            double upperValue = sorted.get(upperIndex);
            double upperWeight = scaledIndex - lowerIndex;
            return lowerValue + (upperValue - lowerValue) * upperWeight;
        }
    }

    private record DiagnosticsSummary(
            String autoName,
            boolean finished,
            double elapsedSec,
            double distanceTraveledMeters,
            int feedSamples,
            int movingFeedSamples,
            double movingFeedFraction,
            double p50MovingAimErrorDeg,
            double p90MovingAimErrorDeg,
            double maxMovingAimErrorDeg,
            double meanMovingAimErrorDeg,
            int descentCrossingSamples,
            int descentWithinIdealSamples,
            double descentWithinIdealFraction,
            double p50DescentMissDistanceMeters,
            double p90DescentMissDistanceMeters,
            double maxDescentMissDistanceMeters,
            double meanDescentMissDistanceMeters,
            double meanSignedDescentMissDistanceMeters,
            double meanAlongVelocityMissDistanceMeters,
            double meanAbsAlongVelocityMissDistanceMeters,
            double p90AbsAlongVelocityMissDistanceMeters,
            double meanAbsCrossVelocityMissDistanceMeters,
            double p90AbsCrossVelocityMissDistanceMeters,
            double meanAbsVelocityToTargetAngleDeg,
            double p90AbsVelocityToTargetAngleDeg,
            double meanMovingFeedTargetRpm,
            double meanMovingFeedMeasuredRpm,
            double meanMovingFeedRpmError,
            double p90AbsMovingFeedRpmError,
            double meanMovingFeedFieldVelocityEstimateErrorMps,
            double p90MovingFeedFieldVelocityEstimateErrorMps,
            double meanMovingFeedOmegaEstimateErrorRadPerSec,
            double p90MovingFeedOmegaEstimateErrorRadPerSec,
            String descentSampleDetails) {
        private double p50DescentMissDistanceInches() {
            return p50DescentMissDistanceMeters * 39.37007874;
        }

        private double p90DescentMissDistanceInches() {
            return p90DescentMissDistanceMeters * 39.37007874;
        }

        private double maxDescentMissDistanceInches() {
            return maxDescentMissDistanceMeters * 39.37007874;
        }

        private double meanDescentMissDistanceInches() {
            return meanDescentMissDistanceMeters * 39.37007874;
        }

        private String toSingleLineSummary(Path summaryFile) {
            return String.format(
                    Locale.US,
                    "[AutoAimDiagnostics] auto=%s finished=%s elapsed=%.2fs distance=%.2fm movingFeedSamples=%d p90Aim=%.3fdeg p90DescentMiss=%.3fm(%.2fin) maxDescentMiss=%.3fm(%.2fin) summary=%s",
                    autoName,
                    finished,
                    elapsedSec,
                    distanceTraveledMeters,
                    movingFeedSamples,
                    p90MovingAimErrorDeg,
                    p90DescentMissDistanceMeters,
                    p90DescentMissDistanceInches(),
                    maxDescentMissDistanceMeters,
                    maxDescentMissDistanceInches(),
                    summaryFile);
        }

        private String toMultilineString(DiagnosticsConfig config) {
            return String.format(
                    Locale.US,
                    """
                    auto_name=%s
                    finished=%s
                    elapsed_sec=%.3f
                    distance_traveled_m=%.3f
                    feed_samples=%d
                    moving_feed_samples=%d
                    moving_feed_fraction=%.5f
                    p50_moving_aim_error_deg=%.5f
                    p90_moving_aim_error_deg=%.5f
                    max_moving_aim_error_deg=%.5f
                    mean_moving_aim_error_deg=%.5f
                    mean_moving_feed_target_rpm=%.5f
                    mean_moving_feed_measured_rpm=%.5f
                    mean_moving_feed_rpm_error=%.5f
                    p90_abs_moving_feed_rpm_error=%.5f
                    mean_moving_feed_field_velocity_estimate_error_mps=%.5f
                    p90_moving_feed_field_velocity_estimate_error_mps=%.5f
                    mean_moving_feed_omega_estimate_error_rad_per_sec=%.5f
                    p90_moving_feed_omega_estimate_error_rad_per_sec=%.5f
                    descent_crossing_samples=%d
                    descent_within_ideal_samples=%d
                    descent_within_ideal_fraction=%.5f
                    p50_descent_miss_m=%.5f
                    p90_descent_miss_m=%.5f
                    max_descent_miss_m=%.5f
                    mean_descent_miss_m=%.5f
                    mean_signed_descent_miss_m=%.5f
                    mean_along_velocity_miss_m=%.5f
                    mean_abs_along_velocity_miss_m=%.5f
                    p90_abs_along_velocity_miss_m=%.5f
                    mean_abs_cross_velocity_miss_m=%.5f
                    p90_abs_cross_velocity_miss_m=%.5f
                    mean_abs_velocity_to_target_angle_deg=%.5f
                    p90_abs_velocity_to_target_angle_deg=%.5f
                    descent_sample_details=%s
                    p50_descent_miss_in=%.5f
                    p90_descent_miss_in=%.5f
                    max_descent_miss_in=%.5f
                    mean_descent_miss_in=%.5f
                    mean_signed_descent_miss_in=%.5f
                    ideal_descent_miss_m=%.5f
                    ideal_descent_miss_in=%.5f
                    threshold_min_distance_m=%.3f
                    threshold_min_moving_feed_samples=%d
                    threshold_min_moving_feed_fraction=%.5f
                    threshold_max_p90_deg=%.5f
                    threshold_max_deg=%.5f
                    threshold_min_descent_crossing_samples=%d
                    threshold_max_p90_descent_m=%.5f
                    threshold_max_descent_m=%.5f
                    threshold_min_descent_within_ideal_fraction=%.5f
                    """,
                    autoName,
                    finished,
                    elapsedSec,
                    distanceTraveledMeters,
                    feedSamples,
                    movingFeedSamples,
                    movingFeedFraction,
                    p50MovingAimErrorDeg,
                    p90MovingAimErrorDeg,
                    maxMovingAimErrorDeg,
                    meanMovingAimErrorDeg,
                    meanMovingFeedTargetRpm,
                    meanMovingFeedMeasuredRpm,
                    meanMovingFeedRpmError,
                    p90AbsMovingFeedRpmError,
                    meanMovingFeedFieldVelocityEstimateErrorMps,
                    p90MovingFeedFieldVelocityEstimateErrorMps,
                    meanMovingFeedOmegaEstimateErrorRadPerSec,
                    p90MovingFeedOmegaEstimateErrorRadPerSec,
                    descentCrossingSamples,
                    descentWithinIdealSamples,
                    descentWithinIdealFraction,
                    p50DescentMissDistanceMeters,
                    p90DescentMissDistanceMeters,
                    maxDescentMissDistanceMeters,
                    meanDescentMissDistanceMeters,
                    meanSignedDescentMissDistanceMeters,
                    meanAlongVelocityMissDistanceMeters,
                    meanAbsAlongVelocityMissDistanceMeters,
                    p90AbsAlongVelocityMissDistanceMeters,
                    meanAbsCrossVelocityMissDistanceMeters,
                    p90AbsCrossVelocityMissDistanceMeters,
                    meanAbsVelocityToTargetAngleDeg,
                    p90AbsVelocityToTargetAngleDeg,
                    descentSampleDetails,
                    p50DescentMissDistanceInches(),
                    p90DescentMissDistanceInches(),
                    maxDescentMissDistanceInches(),
                    meanDescentMissDistanceInches(),
                    meanSignedDescentMissDistanceMeters * 39.37007874,
                    config.idealDescentMissDistanceMeters(),
                    config.idealDescentMissDistanceMeters() * 39.37007874,
                    config.minDistanceTraveledMeters(),
                    config.minMovingFeedSamples(),
                    config.minMovingFeedFraction(),
                    config.maxP90MovingAimErrorDeg(),
                    config.maxMovingAimErrorDeg(),
                    config.minDescentCrossingSamples(),
                    config.maxP90DescentMissDistanceMeters(),
                    config.maxDescentMissDistanceMeters(),
                    config.minDescentWithinIdealFraction());
        }
    }
}
