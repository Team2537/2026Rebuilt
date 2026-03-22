package frc.robot.diagnostics;

import static org.junit.jupiter.api.Assertions.assertFalse;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.util.struct.StructBuffer;
import frc.robot.generated.TunerConstants;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import org.junit.jupiter.api.Test;

class MdbetDriveLogAnalyzerTest {
    private static final Pattern QUAL_LOG_PATTERN = Pattern.compile(".*_q(\\d+)\\.wpilog$");
    private static final String[] MODULE_NAMES = {"FL", "FR", "BL", "BR"};
    private static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.967);
    private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY));

    @Test
    void analyzeMdbetDriveLogsAfterMatch7() throws Exception {
        Path dir = Path.of("logs/mdbet").toAbsolutePath();
        List<Path> logs = Files.list(dir)
                .filter(Files::isRegularFile)
                .filter(path -> matchNumber(path.getFileName().toString()).orElse(-1) > 7)
                .sorted(Comparator.comparingInt(path -> matchNumber(path.getFileName().toString()).orElse(Integer.MAX_VALUE)))
                .toList();

        assertFalse(logs.isEmpty(), "No qualification logs after match 7 found in " + dir);

        List<LogReport> reports = new ArrayList<>();
        for (Path log : logs) {
            reports.add(analyze(log));
        }

        String report = formatCombinedReport(reports);
        Path out = Path.of("build/reports/diagnostics/mdbet-drive-after-q7.txt").toAbsolutePath();
        Files.createDirectories(out.getParent());
        Files.writeString(out, report);

        System.out.println(report);
        System.out.println("Drive log analysis written: " + out);
    }

    private static LogReport analyze(Path wpilog) throws IOException {
        DataLogReader reader = new DataLogReader(wpilog.toString());
        if (!reader.isValid()) {
            throw new IllegalStateException("Invalid WPILOG: " + wpilog);
        }

        List<DataLogRecord> records = recordsUntilFailure(reader);
        Map<Integer, EntryInfo> entries = new HashMap<>();
        for (DataLogRecord record : records) {
            if (!record.isStart()) {
                continue;
            }
            var start = record.getStartData();
            entries.put(start.entry, new EntryInfo(start.name, start.type));
        }

        EntryIds ids = EntryIds.from(entries);
        Analyzer analyzer = new Analyzer(entries, ids);
        for (DataLogRecord record : records) {
            analyzer.accept(record);
        }
        return analyzer.finish(wpilog);
    }

    private static String formatCombinedReport(List<LogReport> reports) {
        StringBuilder out = new StringBuilder();
        out.append("MDBET drivetrain/module log analysis (qualification matches after q7)\n");
        out.append("logs_analyzed=").append(reports.size()).append('\n');
        out.append("matches=")
                .append(reports.stream().map(report -> report.matchLabel).collect(Collectors.joining(", ")))
                .append("\n\n");

        CombinedReport combined = CombinedReport.from(reports);
        out.append(combined.summary()).append("\n");

        for (LogReport report : reports) {
            out.append(report.format()).append("\n");
        }
        return out.toString();
    }

    private static Optional<Integer> matchNumber(String fileName) {
        Matcher matcher = QUAL_LOG_PATTERN.matcher(fileName);
        if (!matcher.matches()) {
            return Optional.empty();
        }
        return Optional.of(Integer.parseInt(matcher.group(1)));
    }

    private record EntryInfo(String name, String type) {}

    private static final class EntryIds {
        private int mode = -1;
        private int enabled = -1;
        private int matchTime = -1;
        private int batteryVoltage = -1;
        private int batteryLow = -1;
        private int brownout = -1;
        private int gyroConnected = -1;
        private int canBusUtilization = -1;
        private int canBusOffCount = -1;
        private int canRec = -1;
        private int canTec = -1;
        private int canTxFull = -1;
        private int canStatus = -1;
        private int odomModuleSampleCounts = -1;
        private int odomGyroSampleCount = -1;
        private int odomSharedSampleCount = -1;
        private int odomSampleCountMismatch = -1;
        private int droppedPhoenixSamples = -1;
        private int droppedTimestampSamples = -1;
        private int requestedSpeeds = -1;
        private int setpointSpeeds = -1;
        private int measuredSpeeds = -1;
        private int measuredModuleStates = -1;
        private int optimizedSetpointStates = -1;
        private int pose = -1;
        private final int[] driveConnected = new int[4];
        private final int[] turnConnected = new int[4];
        private final int[] encoderConnected = new int[4];
        private final int[] drivePositionRad = new int[4];
        private final int[] driveVelocityRadPerSec = new int[4];
        private final int[] driveAppliedVolts = new int[4];
        private final int[] driveCurrentAmps = new int[4];
        private final int[] turnAbsolutePosition = new int[4];
        private final int[] turnPosition = new int[4];
        private final int[] turnVelocityRadPerSec = new int[4];
        private final int[] turnAppliedVolts = new int[4];
        private final int[] turnCurrentAmps = new int[4];

        private EntryIds() {
            Arrays.fill(driveConnected, -1);
            Arrays.fill(turnConnected, -1);
            Arrays.fill(encoderConnected, -1);
            Arrays.fill(drivePositionRad, -1);
            Arrays.fill(driveVelocityRadPerSec, -1);
            Arrays.fill(driveAppliedVolts, -1);
            Arrays.fill(driveCurrentAmps, -1);
            Arrays.fill(turnAbsolutePosition, -1);
            Arrays.fill(turnPosition, -1);
            Arrays.fill(turnVelocityRadPerSec, -1);
            Arrays.fill(turnAppliedVolts, -1);
            Arrays.fill(turnCurrentAmps, -1);
        }

        private static EntryIds from(Map<Integer, EntryInfo> entries) {
            EntryIds ids = new EntryIds();
            ids.mode = findEntry(entries, "RobotState/Mode");
            ids.enabled = findEntry(entries, "RobotState/Enabled");
            ids.matchTime = findEntry(entries, "RobotState/MatchTime");
            ids.batteryVoltage = findEntry(entries, "RobotState/BatteryVoltage", "PowerDistribution/totalVoltageVolts", "SystemStats/BatteryVoltage");
            ids.batteryLow = findEntry(entries, "RobotState/BatteryLow");
            ids.brownout = findEntry(entries, "SystemStats/BrownedOut");
            ids.gyroConnected = findEntry(entries, "RobotState/GyroConnected");
            ids.canBusUtilization = findEntry(entries, "CAN/CANivore/BusUtilization");
            ids.canBusOffCount = findEntry(entries, "CAN/CANivore/BusOffCount");
            ids.canRec = findEntry(entries, "CAN/CANivore/RecieveErrorCount", "CAN/CANivore/ReceiveErrorCount");
            ids.canTec = findEntry(entries, "CAN/CANivore/TransmitErrorCount");
            ids.canTxFull = findEntry(entries, "CAN/CANivore/TxFullCount");
            ids.canStatus = findEntry(entries, "CAN/CANivore/Status");
            ids.odomModuleSampleCounts = findEntry(entries, "Drive/Odometry/ModuleSampleCounts");
            ids.odomGyroSampleCount = findEntry(entries, "Drive/Odometry/GyroSampleCount");
            ids.odomSharedSampleCount = findEntry(entries, "Drive/Odometry/SharedSampleCount");
            ids.odomSampleCountMismatch = findEntry(entries, "Drive/Odometry/SampleCountMismatch");
            ids.droppedPhoenixSamples = findEntry(entries, "Drive/OdometryThread/DroppedPhoenixSamples");
            ids.droppedTimestampSamples = findEntry(entries, "Drive/OdometryThread/DroppedTimestampSamples");
            ids.requestedSpeeds = findEntry(entries, "SwerveChassisSpeeds/Requested");
            ids.setpointSpeeds = findEntry(entries, "SwerveChassisSpeeds/Setpoints");
            ids.measuredSpeeds = findEntry(entries, "SwerveChassisSpeeds/Measured");
            ids.measuredModuleStates = findEntry(entries, "SwerveStates/Measured");
            ids.optimizedSetpointStates = findEntry(entries, "SwerveStates/SetpointsOptimized");
            ids.pose = findEntry(entries, "Odometry/Robot");
            for (int i = 0; i < 4; i++) {
                ids.driveConnected[i] = findEntry(entries, "Drive/Module" + i + "/DriveConnected");
                ids.turnConnected[i] = findEntry(entries, "Drive/Module" + i + "/TurnConnected");
                ids.encoderConnected[i] = findEntry(entries, "Drive/Module" + i + "/TurnEncoderConnected");
                ids.drivePositionRad[i] = findEntry(entries, "Drive/Module" + i + "/DrivePositionRad");
                ids.driveVelocityRadPerSec[i] = findEntry(entries, "Drive/Module" + i + "/DriveVelocityRadPerSec");
                ids.driveAppliedVolts[i] = findEntry(entries, "Drive/Module" + i + "/DriveAppliedVolts");
                ids.driveCurrentAmps[i] = findEntry(entries, "Drive/Module" + i + "/DriveCurrentAmps");
                ids.turnAbsolutePosition[i] = findEntry(entries, "Drive/Module" + i + "/TurnAbsolutePosition");
                ids.turnPosition[i] = findEntry(entries, "Drive/Module" + i + "/TurnPosition");
                ids.turnVelocityRadPerSec[i] = findEntry(entries, "Drive/Module" + i + "/TurnVelocityRadPerSec");
                ids.turnAppliedVolts[i] = findEntry(entries, "Drive/Module" + i + "/TurnAppliedVolts");
                ids.turnCurrentAmps[i] = findEntry(entries, "Drive/Module" + i + "/TurnCurrentAmps");
            }
            return ids;
        }
    }

    private static int findEntry(Map<Integer, EntryInfo> entries, String... suffixes) {
        for (String suffix : suffixes) {
            String target = normalize(suffix);
            for (var entry : entries.entrySet()) {
                if (normalize(entry.getValue().name).endsWith(target)) {
                    return entry.getKey();
                }
            }
        }
        return -1;
    }

    private static String normalize(String value) {
        return value.toLowerCase(Locale.ROOT).replace("\\", "/");
    }

    private static final class Analyzer {
        private final Map<Integer, EntryInfo> entries;
        private final EntryIds ids;
        private final StructBuffer<ChassisSpeeds> speedsBuf = StructBuffer.create(ChassisSpeeds.struct);
        private final StructBuffer<Pose2d> poseBuf = StructBuffer.create(Pose2d.struct);
        private final StructBuffer<Rotation2d> rotationBuf = StructBuffer.create(Rotation2d.struct);
        private final StructBuffer<SwerveModuleState> stateBuf = StructBuffer.create(SwerveModuleState.struct);
        private final State state = new State();
        private final Summary summary = new Summary();

        private Analyzer(Map<Integer, EntryInfo> entries, EntryIds ids) {
            this.entries = entries;
            this.ids = ids;
        }

        private void accept(DataLogRecord record) {
            if (record.isStart() || record.isControl()) {
                return;
            }

            double timestampSec = record.getTimestamp() / 1_000_000.0;
            summary.lastTimestampSec = Math.max(summary.lastTimestampSec, timestampSec);
            int entry = record.getEntry();

            if (entry == ids.mode) {
                state.mode = readStringLenient(record, state.mode);
            } else if (entry == ids.enabled) {
                state.enabled = readBooleanLenient(record, entryName(entry));
            } else if (entry == ids.matchTime) {
                state.matchTime = readDoubleLenient(record);
            } else if (entry == ids.batteryVoltage) {
                state.batteryVoltage = readDoubleLenient(record);
            } else if (entry == ids.batteryLow) {
                boolean newValue = readBooleanLenient(record, entryName(entry));
                trackBooleanEvent(newValue, state.batteryLow, timestampSec, "batteryLow", summary.stateEvents);
                state.batteryLow = newValue;
            } else if (entry == ids.brownout) {
                boolean newValue = readBooleanLenient(record, entryName(entry));
                trackDowntime("brownout", newValue, state.brownout, timestampSec, summary.brownoutTracker, summary.stateEvents);
                state.brownout = newValue;
            } else if (entry == ids.gyroConnected) {
                boolean newValue = readBooleanLenient(record, entryName(entry));
                trackDowntime("gyroConnected", !newValue, !state.gyroConnected, timestampSec, summary.gyroTracker, summary.connectionEvents);
                state.gyroConnected = newValue;
            } else if (entry == ids.canBusUtilization) {
                state.canBusUtilization = readDoubleLenient(record);
                summary.canUtilization.add(state.canBusUtilization);
            } else if (entry == ids.canBusOffCount) {
                long newValue = readLongLenient(record);
                trackCounterIncrease("CAN busOffCount", newValue, state.canBusOffCount, timestampSec, summary.canEvents);
                state.canBusOffCount = newValue;
                summary.maxCanBusOffCount = Math.max(summary.maxCanBusOffCount, newValue);
            } else if (entry == ids.canRec) {
                long newValue = readLongLenient(record);
                trackCounterIncrease("CAN receiveErrorCount", newValue, state.canRec, timestampSec, summary.canEvents);
                state.canRec = newValue;
                summary.maxCanRec = Math.max(summary.maxCanRec, newValue);
            } else if (entry == ids.canTec) {
                long newValue = readLongLenient(record);
                trackCounterIncrease("CAN transmitErrorCount", newValue, state.canTec, timestampSec, summary.canEvents);
                state.canTec = newValue;
                summary.maxCanTec = Math.max(summary.maxCanTec, newValue);
            } else if (entry == ids.canTxFull) {
                long newValue = readLongLenient(record);
                trackCounterIncrease("CAN txFullCount", newValue, state.canTxFull, timestampSec, summary.canEvents);
                state.canTxFull = newValue;
                summary.maxCanTxFull = Math.max(summary.maxCanTxFull, newValue);
            } else if (entry == ids.canStatus) {
                String newValue = readStringLenient(record, state.canStatus);
                if (!newValue.equals(state.canStatus)) {
                    summary.canEvents.add(String.format(Locale.US, "t=%.3f CAN status=%s", timestampSec, newValue));
                }
                state.canStatus = newValue;
                summary.finalCanStatus = newValue;
            } else if (entry == ids.odomModuleSampleCounts) {
                state.moduleSampleCounts = readIntArrayLenient(record, 4);
            } else if (entry == ids.odomGyroSampleCount) {
                state.gyroSampleCount = (int) readLongLenient(record);
            } else if (entry == ids.odomSharedSampleCount) {
                state.sharedSampleCount = (int) readLongLenient(record);
            } else if (entry == ids.odomSampleCountMismatch) {
                boolean newValue = readBooleanLenient(record, entryName(entry));
                trackDowntime("odomSampleCountMismatch", newValue, state.odomSampleCountMismatch, timestampSec, summary.odomMismatchTracker, summary.odomEvents);
                state.odomSampleCountMismatch = newValue;
            } else if (entry == ids.droppedPhoenixSamples) {
                long newValue = readLongLenient(record);
                trackCounterIncrease("DroppedPhoenixSamples", newValue, state.droppedPhoenixSamples, timestampSec, summary.odomEvents);
                state.droppedPhoenixSamples = newValue;
            } else if (entry == ids.droppedTimestampSamples) {
                long newValue = readLongLenient(record);
                trackCounterIncrease("DroppedTimestampSamples", newValue, state.droppedTimestampSamples, timestampSec, summary.odomEvents);
                state.droppedTimestampSamples = newValue;
            } else if (entry == ids.requestedSpeeds) {
                state.requestedSpeeds = readSpeeds(record, state.requestedSpeeds);
            } else if (entry == ids.setpointSpeeds) {
                state.setpointSpeeds = readSpeeds(record, state.setpointSpeeds);
            } else if (entry == ids.measuredModuleStates) {
                state.measuredModuleStates = readModuleStates(record, state.measuredModuleStates);
            } else if (entry == ids.optimizedSetpointStates) {
                state.optimizedSetpointStates = readModuleStates(record, state.optimizedSetpointStates);
            } else if (entry == ids.pose) {
                state.pose = readPose(record, state.pose);
            } else if (entry == ids.measuredSpeeds) {
                state.measuredSpeeds = readSpeeds(record, state.measuredSpeeds);
                sample(timestampSec);
            } else {
                for (int i = 0; i < 4; i++) {
                    if (entry == ids.driveConnected[i]) {
                        boolean newValue = readBooleanLenient(record, entryName(entry));
                        trackDowntime(MODULE_NAMES[i] + " driveConnected", !newValue, !state.modules[i].driveConnected,
                                timestampSec, summary.moduleSummaries[i].driveTracker, summary.connectionEvents);
                        state.modules[i].driveConnected = newValue;
                        return;
                    }
                    if (entry == ids.turnConnected[i]) {
                        boolean newValue = readBooleanLenient(record, entryName(entry));
                        trackDowntime(MODULE_NAMES[i] + " turnConnected", !newValue, !state.modules[i].turnConnected,
                                timestampSec, summary.moduleSummaries[i].turnTracker, summary.connectionEvents);
                        state.modules[i].turnConnected = newValue;
                        return;
                    }
                    if (entry == ids.encoderConnected[i]) {
                        boolean newValue = readBooleanLenient(record, entryName(entry));
                        trackDowntime(MODULE_NAMES[i] + " encoderConnected", !newValue, !state.modules[i].encoderConnected,
                                timestampSec, summary.moduleSummaries[i].encoderTracker, summary.connectionEvents);
                        state.modules[i].encoderConnected = newValue;
                        return;
                    }
                    if (entry == ids.drivePositionRad[i]) {
                        state.modules[i].drivePositionRad = readDoubleLenient(record);
                        return;
                    }
                    if (entry == ids.driveVelocityRadPerSec[i]) {
                        state.modules[i].driveVelocityMps = readDoubleLenient(record) * WHEEL_RADIUS_METERS;
                        return;
                    }
                    if (entry == ids.driveAppliedVolts[i]) {
                        state.modules[i].driveAppliedVolts = readDoubleLenient(record);
                        return;
                    }
                    if (entry == ids.driveCurrentAmps[i]) {
                        state.modules[i].driveCurrentAmps = readDoubleLenient(record);
                        return;
                    }
                    if (entry == ids.turnAbsolutePosition[i]) {
                        state.modules[i].turnAbsolutePosition = readRotation(record, state.modules[i].turnAbsolutePosition);
                        return;
                    }
                    if (entry == ids.turnPosition[i]) {
                        state.modules[i].turnPosition = readRotation(record, state.modules[i].turnPosition);
                        return;
                    }
                    if (entry == ids.turnVelocityRadPerSec[i]) {
                        state.modules[i].turnVelocityRadPerSec = readDoubleLenient(record);
                        return;
                    }
                    if (entry == ids.turnAppliedVolts[i]) {
                        state.modules[i].turnAppliedVolts = readDoubleLenient(record);
                        return;
                    }
                    if (entry == ids.turnCurrentAmps[i]) {
                        state.modules[i].turnCurrentAmps = readDoubleLenient(record);
                        return;
                    }
                }
            }
        }

        private void sample(double timestampSec) {
            if (Double.isFinite(summary.lastSampleTimestampSec)) {
                double dt = clampDt(timestampSec - summary.lastSampleTimestampSec);
                summary.sampleDt.add(dt);
                if (summary.lastSampleEnabled) {
                    summary.enabledTimeSec += dt;
                    if ("AUTO".equals(summary.lastSampleMode)) {
                        summary.autoTimeSec += dt;
                    } else if ("TELEOP".equals(summary.lastSampleMode)) {
                        summary.teleopTimeSec += dt;
                    }
                } else {
                    summary.disabledTimeSec += dt;
                }
            }
            summary.lastSampleTimestampSec = timestampSec;
            summary.lastSampleEnabled = state.enabled;
            summary.lastSampleMode = state.mode;

            summary.totalMeasuredSamples++;
            if (!state.enabled) {
                return;
            }

            summary.enabledMeasuredSamples++;
            summary.voltageEnabled.add(state.batteryVoltage);
            summary.sharedSampleCount.add(state.sharedSampleCount);
            summary.gyroSampleCount.add(state.gyroSampleCount);
            summary.droppedPhoenixSamples.add(state.droppedPhoenixSamples);
            summary.droppedTimestampSamples.add(state.droppedTimestampSamples);

            ChassisSpeeds requested = state.requestedSpeeds != null ? state.requestedSpeeds : new ChassisSpeeds();
            ChassisSpeeds setpoint = state.setpointSpeeds != null ? state.setpointSpeeds : requested;
            ChassisSpeeds measured = state.measuredSpeeds != null ? state.measuredSpeeds : new ChassisSpeeds();

            double reqLin = Math.hypot(requested.vxMetersPerSecond, requested.vyMetersPerSecond);
            double setLin = Math.hypot(setpoint.vxMetersPerSecond, setpoint.vyMetersPerSecond);
            double measLin = Math.hypot(measured.vxMetersPerSecond, measured.vyMetersPerSecond);
            double translationVectorError = Math.hypot(
                    setpoint.vxMetersPerSecond - measured.vxMetersPerSecond,
                    setpoint.vyMetersPerSecond - measured.vyMetersPerSecond);
            double omegaError = Math.abs(setpoint.omegaRadiansPerSecond - measured.omegaRadiansPerSecond);

            summary.requestedLinearSpeed.add(reqLin);
            summary.setpointLinearSpeed.add(setLin);
            summary.measuredLinearSpeed.add(measLin);
            summary.translationVectorError.add(translationVectorError);
            summary.omegaError.add(omegaError);

            if (setLin > 0.30) {
                summary.activeTranslationSamples++;
                double ratio = measLin / Math.max(0.05, setLin);
                summary.translationRatio.add(ratio);
                if (measLin < Math.max(0.65 * setLin, setLin - 1.00)) {
                    summary.translationLagFindings.add(
                            (setLin - measLin) + translationVectorError,
                            snapshot(timestampSec, String.format(Locale.US,
                                    "translation lag set=%.2f m/s meas=%.2f req=%.2f err=%.2f omegaErr=%.2f V=%.2f can=%.0f%% mode=%s match=%.1f",
                                    setLin, measLin, reqLin, translationVectorError, omegaError,
                                    state.batteryVoltage, state.canBusUtilization * 100.0, state.mode, state.matchTime)));
                }
            }

            if (Math.abs(setpoint.omegaRadiansPerSecond) > 0.75) {
                summary.activeOmegaSamples++;
                double ratio = Math.abs(measured.omegaRadiansPerSecond) / Math.max(0.05, Math.abs(setpoint.omegaRadiansPerSecond));
                summary.omegaRatio.add(ratio);
                if (Math.abs(measured.omegaRadiansPerSecond) < Math.max(0.55 * Math.abs(setpoint.omegaRadiansPerSecond),
                        Math.abs(setpoint.omegaRadiansPerSecond) - 1.5)) {
                    summary.rotationLagFindings.add(
                            Math.abs(setpoint.omegaRadiansPerSecond) - Math.abs(measured.omegaRadiansPerSecond),
                            snapshot(timestampSec, String.format(Locale.US,
                                    "rotation lag set=%.2f rad/s meas=%.2f setLin=%.2f measLin=%.2f V=%.2f mode=%s match=%.1f",
                                    setpoint.omegaRadiansPerSecond, measured.omegaRadiansPerSecond,
                                    setLin, measLin, state.batteryVoltage, state.mode, state.matchTime)));
                }
            }

            if (setLin < 0.05 && Math.abs(setpoint.omegaRadiansPerSecond) < 0.10) {
                summary.stationarySamples++;
                summary.stationaryCreep.add(measLin);
                if (measLin > 0.25) {
                    summary.stationaryFindings.add(
                            measLin,
                            snapshot(timestampSec, String.format(Locale.US,
                                    "unexpected motion while commanded stopped meas=%.2f m/s omega=%.2f rad/s V=%.2f mode=%s match=%.1f pose=%s",
                                    measLin, measured.omegaRadiansPerSecond, state.batteryVoltage,
                                    state.mode, state.matchTime, poseString(state.pose))));
                }
            }

            if (state.batteryVoltage < 9.5) {
                summary.lowVoltageFindings.add(
                        12.5 - state.batteryVoltage,
                        snapshot(timestampSec, String.format(Locale.US,
                                "low voltage V=%.2f set=%.2f meas=%.2f omegaSet=%.2f can=%.0f%% brownout=%s mode=%s match=%.1f",
                                state.batteryVoltage, setLin, measLin, setpoint.omegaRadiansPerSecond,
                                state.canBusUtilization * 100.0, state.brownout, state.mode, state.matchTime)));
            }

            if (state.canBusUtilization > 0.85 || state.canBusOffCount > 0 || state.canRec > 0 || state.canTec > 0 || state.canTxFull > 0) {
                summary.canFindings.add(
                        state.canBusUtilization + state.canBusOffCount + state.canRec + state.canTec + state.canTxFull,
                        snapshot(timestampSec, String.format(Locale.US,
                                "CAN stress util=%.1f%% busOff=%d REC=%d TEC=%d txFull=%d status=%s mode=%s match=%.1f",
                                state.canBusUtilization * 100.0, state.canBusOffCount, state.canRec, state.canTec,
                                state.canTxFull, state.canStatus, state.mode, state.matchTime)));
            }

            if (state.odomSampleCountMismatch || state.sharedSampleCount < 1 || state.droppedPhoenixSamples > 0 || state.droppedTimestampSamples > 0) {
                summary.odomFindings.add(
                        (state.odomSampleCountMismatch ? 100.0 : 0.0) + Math.max(0, 4 - state.sharedSampleCount)
                                + state.droppedPhoenixSamples + state.droppedTimestampSamples,
                        snapshot(timestampSec, String.format(Locale.US,
                                "odometry issue mismatch=%s shared=%d gyro=%d moduleCounts=%s droppedPhoenix=%d droppedTs=%d mode=%s match=%.1f",
                                state.odomSampleCountMismatch, state.sharedSampleCount, state.gyroSampleCount,
                                Arrays.toString(state.moduleSampleCounts), state.droppedPhoenixSamples,
                                state.droppedTimestampSamples, state.mode, state.matchTime)));
            }

            SwerveModuleState[] setpointStates = state.optimizedSetpointStates;
            if (setpointStates == null || setpointStates.length != 4) {
                setpointStates = KINEMATICS.toSwerveModuleStates(setpoint);
            }
            SwerveModuleState[] measuredStates = state.measuredModuleStates;
            if (measuredStates == null || measuredStates.length != 4) {
                measuredStates = new SwerveModuleState[4];
                for (int i = 0; i < 4; i++) {
                    measuredStates[i] = new SwerveModuleState(state.modules[i].driveVelocityMps, state.modules[i].turnPosition);
                }
            }

            double[] absMeasuredSpeeds = new double[4];
            double avgDemand = 0.0;
            for (int i = 0; i < 4; i++) {
                ModuleState module = state.modules[i];
                ModuleSummary moduleSummary = summary.moduleSummaries[i];
                moduleSummary.driveCurrent.add(module.driveCurrentAmps);
                moduleSummary.driveVolts.add(Math.abs(module.driveAppliedVolts));
                moduleSummary.turnCurrent.add(module.turnCurrentAmps);
                moduleSummary.turnVolts.add(Math.abs(module.turnAppliedVolts));
                moduleSummary.turnVelocity.add(Math.abs(module.turnVelocityRadPerSec));
                moduleSummary.absRelTurnDeltaDeg.add(Math.abs(angleErrorDeg(module.turnAbsolutePosition, module.turnPosition)));

                double absSetSpeed = Math.abs(setpointStates[i].speedMetersPerSecond);
                double absMeasuredSpeed = Math.abs(measuredStates[i].speedMetersPerSecond);
                absMeasuredSpeeds[i] = absMeasuredSpeed;
                avgDemand += absSetSpeed / 4.0;
                if (absSetSpeed > 0.30) {
                    moduleSummary.activeSpeedRatio.add(absMeasuredSpeed / Math.max(0.05, absSetSpeed));
                    moduleSummary.activeSpeedError.add(Math.abs(absSetSpeed - absMeasuredSpeed));
                }
                double angleErrDeg = Math.abs(angleErrorDeg(setpointStates[i].angle, module.turnPosition));
                if (absSetSpeed > 0.50 || absMeasuredSpeed > 0.50) {
                    moduleSummary.angleErrorDeg.add(angleErrDeg);
                }
                if (absSetSpeed > 1.50 && absMeasuredSpeed < Math.max(0.60 * absSetSpeed, absSetSpeed - 1.00)) {
                    summary.moduleLagFindings.add(
                            absSetSpeed - absMeasuredSpeed,
                            snapshot(timestampSec, String.format(Locale.US,
                                    "%s speed lag set=%.2f m/s meas=%.2f angleErr=%.1f deg driveI=%.1fA driveV=%.1fV turnI=%.1fA Vbat=%.2f pose=%s mode=%s match=%.1f",
                                    MODULE_NAMES[i], absSetSpeed, absMeasuredSpeed, angleErrDeg,
                                    module.driveCurrentAmps, module.driveAppliedVolts, module.turnCurrentAmps,
                                    state.batteryVoltage, poseString(state.pose), state.mode, state.matchTime)));
                }
                if (angleErrDeg > 15.0 && absSetSpeed > 1.0) {
                    summary.moduleAngleFindings.add(
                            angleErrDeg,
                            snapshot(timestampSec, String.format(Locale.US,
                                    "%s angle error=%.1f deg setSpeed=%.2f measSpeed=%.2f turnVel=%.2f abs-vs-rel=%.1f deg pose=%s mode=%s match=%.1f",
                                    MODULE_NAMES[i], angleErrDeg, absSetSpeed, absMeasuredSpeed,
                                    module.turnVelocityRadPerSec,
                                    Math.abs(angleErrorDeg(module.turnAbsolutePosition, module.turnPosition)),
                                    poseString(state.pose), state.mode, state.matchTime)));
                }
            }

            if (avgDemand > 1.5) {
                double medianSpeed = median(absMeasuredSpeeds.clone());
                double minRatio = Double.POSITIVE_INFINITY;
                int minModule = -1;
                for (int i = 0; i < 4; i++) {
                    double ratio = absMeasuredSpeeds[i] / Math.max(0.05, medianSpeed);
                    summary.moduleSummaries[i].peerMedianSpeedRatio.add(ratio);
                    if (ratio < minRatio) {
                        minRatio = ratio;
                        minModule = i;
                    }
                }
                if (minRatio < 0.75) {
                    summary.balanceFindings.add(
                            1.0 - minRatio,
                            snapshot(timestampSec, String.format(Locale.US,
                                    "module balance issue weakest=%s ratioToPeerMedian=%.2f measured=%s setpointAvg=%.2f V=%.2f mode=%s match=%.1f",
                                    MODULE_NAMES[minModule], minRatio, Arrays.toString(round(absMeasuredSpeeds)), avgDemand,
                                    state.batteryVoltage, state.mode, state.matchTime)));
                }
            }
        }

        private LogReport finish(Path wpilog) {
            summary.close(summary.lastTimestampSec);
            return new LogReport(wpilog, summary, entries, ids);
        }

        private String entryName(int entry) {
            EntryInfo info = entries.get(entry);
            return info != null ? info.name : ("entry=" + entry);
        }

        private Pose2d readPose(DataLogRecord record, Pose2d fallback) {
            try {
                return poseBuf.read(record.getRaw());
            } catch (Exception ignored) {
            }
            return fallback;
        }

        private ChassisSpeeds readSpeeds(DataLogRecord record, ChassisSpeeds fallback) {
            try {
                return speedsBuf.read(record.getRaw());
            } catch (Exception ignored) {
            }
            try {
                double[] values = record.getDoubleArray();
                if (values.length >= 3) {
                    return new ChassisSpeeds(values[0], values[1], values[2]);
                }
            } catch (Exception ignored) {
            }
            return fallback;
        }

        private SwerveModuleState[] readModuleStates(DataLogRecord record, SwerveModuleState[] fallback) {
            try {
                return stateBuf.readArray(record.getRaw());
            } catch (Exception ignored) {
            }
            return fallback;
        }

        private Rotation2d readRotation(DataLogRecord record, Rotation2d fallback) {
            try {
                return rotationBuf.read(record.getRaw());
            } catch (Exception ignored) {
            }
            try {
                return Rotation2d.fromRadians(record.getDouble());
            } catch (Exception ignored) {
            }
            return fallback;
        }
    }

    private static void trackCounterIncrease(String label, long newValue, long oldValue, double timestampSec, List<String> out) {
        if (newValue > oldValue) {
            out.add(String.format(Locale.US, "t=%.3f %s increased %d -> %d", timestampSec, label, oldValue, newValue));
        }
    }

    private static void trackBooleanEvent(boolean newValue, boolean oldValue, double timestampSec, String label, List<String> out) {
        if (newValue != oldValue) {
            out.add(String.format(Locale.US, "t=%.3f %s=%s", timestampSec, label, newValue));
        }
    }

    private static void trackDowntime(
            String label,
            boolean newDownState,
            boolean oldDownState,
            double timestampSec,
            DowntimeTracker tracker,
            List<String> out) {
        if (newDownState == oldDownState) {
            return;
        }
        if (newDownState) {
            tracker.begin(timestampSec);
            out.add(String.format(Locale.US, "t=%.3f %s DOWN", timestampSec, label));
        } else {
            double duration = tracker.end(timestampSec);
            out.add(String.format(Locale.US, "t=%.3f %s UP after %.3fs", timestampSec, label, duration));
        }
    }

    private static String snapshot(double timestampSec, String details) {
        return String.format(Locale.US, "t=%.3f %s", timestampSec, details);
    }

    private static String poseString(Pose2d pose) {
        if (pose == null) {
            return "<none>";
        }
        return String.format(Locale.US, "(%.2f,%.2f,%.1fdeg)", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }

    private static double angleErrorDeg(Rotation2d a, Rotation2d b) {
        return Units.radiansToDegrees(MathUtil.angleModulus(a.getRadians() - b.getRadians()));
    }

    private static double readDoubleLenient(DataLogRecord record) {
        try {
            return record.getDouble();
        } catch (Exception ignored) {
        }
        try {
            return record.getFloat();
        } catch (Exception ignored) {
        }
        try {
            return record.getInteger();
        } catch (Exception ignored) {
        }
        return Double.NaN;
    }

    private static long readLongLenient(DataLogRecord record) {
        try {
            return record.getInteger();
        } catch (Exception ignored) {
        }
        try {
            return Math.round(record.getDouble());
        } catch (Exception ignored) {
        }
        return 0L;
    }

    private static boolean readBooleanLenient(DataLogRecord record, String name) {
        try {
            return record.getBoolean();
        } catch (Exception ignored) {
        }
        try {
            return Boolean.parseBoolean(record.getString());
        } catch (Exception ignored) {
        }
        throw new IllegalStateException("Unable to read boolean entry: " + name);
    }

    private static String readStringLenient(DataLogRecord record, String fallback) {
        try {
            return record.getString();
        } catch (Exception ignored) {
        }
        return fallback;
    }

    private static int[] readIntArrayLenient(DataLogRecord record, int expectedLength) {
        int[] values = new int[expectedLength];
        try {
            long[] longs = record.getIntegerArray();
            for (int i = 0; i < Math.min(values.length, longs.length); i++) {
                values[i] = (int) longs[i];
            }
        } catch (Exception ignored) {
        }
        return values;
    }

    private static double clampDt(double dt) {
        if (!Double.isFinite(dt)) {
            return 0.0;
        }
        return Math.max(0.0, Math.min(0.250, dt));
    }

    private static double median(double[] values) {
        Arrays.sort(values);
        if (values.length == 0) {
            return Double.NaN;
        }
        int mid = values.length / 2;
        if ((values.length & 1) == 1) {
            return values[mid];
        }
        return 0.5 * (values[mid - 1] + values[mid]);
    }

    private static String[] round(double[] values) {
        String[] out = new String[values.length];
        for (int i = 0; i < values.length; i++) {
            out[i] = String.format(Locale.US, "%.2f", values[i]);
        }
        return out;
    }

    private static List<DataLogRecord> recordsUntilFailure(DataLogReader reader) {
        List<DataLogRecord> records = new ArrayList<>();
        Iterator<DataLogRecord> iterator = reader.iterator();
        while (true) {
            try {
                if (!iterator.hasNext()) {
                    break;
                }
                records.add(iterator.next());
            } catch (IllegalArgumentException ignored) {
                break;
            }
        }
        return records;
    }

    private static final class State {
        private String mode = "";
        private boolean enabled = false;
        private double matchTime = Double.NaN;
        private double batteryVoltage = Double.NaN;
        private boolean batteryLow = false;
        private boolean brownout = false;
        private boolean gyroConnected = true;
        private double canBusUtilization = Double.NaN;
        private long canBusOffCount = 0;
        private long canRec = 0;
        private long canTec = 0;
        private long canTxFull = 0;
        private String canStatus = "";
        private int[] moduleSampleCounts = new int[] {0, 0, 0, 0};
        private int gyroSampleCount = 0;
        private int sharedSampleCount = 0;
        private boolean odomSampleCountMismatch = false;
        private long droppedPhoenixSamples = 0;
        private long droppedTimestampSamples = 0;
        private ChassisSpeeds requestedSpeeds = new ChassisSpeeds();
        private ChassisSpeeds setpointSpeeds = new ChassisSpeeds();
        private ChassisSpeeds measuredSpeeds = new ChassisSpeeds();
        private SwerveModuleState[] measuredModuleStates = null;
        private SwerveModuleState[] optimizedSetpointStates = null;
        private Pose2d pose = new Pose2d();
        private final ModuleState[] modules = new ModuleState[] {
                new ModuleState(), new ModuleState(), new ModuleState(), new ModuleState()
        };
    }

    private static final class ModuleState {
        private boolean driveConnected = true;
        private boolean turnConnected = true;
        private boolean encoderConnected = true;
        private double drivePositionRad = Double.NaN;
        private double driveVelocityMps = Double.NaN;
        private double driveAppliedVolts = Double.NaN;
        private double driveCurrentAmps = Double.NaN;
        private Rotation2d turnAbsolutePosition = Rotation2d.kZero;
        private Rotation2d turnPosition = Rotation2d.kZero;
        private double turnVelocityRadPerSec = Double.NaN;
        private double turnAppliedVolts = Double.NaN;
        private double turnCurrentAmps = Double.NaN;
    }

    private static final class DowntimeTracker {
        private int transitionsDown = 0;
        private double totalDownSec = 0.0;
        private double longestDownSec = 0.0;
        private double activeDownStartSec = Double.NaN;

        private void begin(double timestampSec) {
            if (Double.isFinite(activeDownStartSec)) {
                return;
            }
            transitionsDown++;
            activeDownStartSec = timestampSec;
        }

        private double end(double timestampSec) {
            if (!Double.isFinite(activeDownStartSec)) {
                return 0.0;
            }
            double duration = Math.max(0.0, timestampSec - activeDownStartSec);
            totalDownSec += duration;
            longestDownSec = Math.max(longestDownSec, duration);
            activeDownStartSec = Double.NaN;
            return duration;
        }

        private void close(double timestampSec) {
            if (Double.isFinite(activeDownStartSec)) {
                end(timestampSec);
            }
        }

        private boolean hadIssue() {
            return transitionsDown > 0 || totalDownSec > 0.0 || Double.isFinite(activeDownStartSec);
        }
    }

    private static final class RunningStats {
        private final List<Double> values = new ArrayList<>();
        private double min = Double.POSITIVE_INFINITY;
        private double max = Double.NEGATIVE_INFINITY;
        private double sum = 0.0;

        private void add(double value) {
            if (!Double.isFinite(value)) {
                return;
            }
            values.add(value);
            min = Math.min(min, value);
            max = Math.max(max, value);
            sum += value;
        }

        private int count() {
            return values.size();
        }

        private double mean() {
            return values.isEmpty() ? Double.NaN : sum / values.size();
        }

        private double min() {
            return values.isEmpty() ? Double.NaN : min;
        }

        private double max() {
            return values.isEmpty() ? Double.NaN : max;
        }

        private double percentile(double percentile) {
            if (values.isEmpty()) {
                return Double.NaN;
            }
            double[] copy = new double[values.size()];
            for (int i = 0; i < values.size(); i++) {
                copy[i] = values.get(i);
            }
            Arrays.sort(copy);
            int index = (int) Math.round((percentile / 100.0) * (copy.length - 1));
            index = Math.max(0, Math.min(copy.length - 1, index));
            return copy[index];
        }

        private String summary(String unit) {
            if (values.isEmpty()) {
                return "<none>";
            }
            return String.format(Locale.US,
                    "n=%d mean=%.3f%s p50=%.3f%s p90=%.3f%s p99=%.3f%s min=%.3f%s max=%.3f%s",
                    count(), mean(), unit, percentile(50), unit, percentile(90), unit, percentile(99), unit, min(), unit, max(), unit);
        }
    }

    private static final class TopFindings {
        private final int limit;
        private final List<Finding> findings = new ArrayList<>();

        private TopFindings(int limit) {
            this.limit = limit;
        }

        private void add(double severity, String detail) {
            findings.add(new Finding(severity, detail));
            findings.sort(Comparator.comparingDouble(Finding::severity).reversed());
            if (findings.size() > limit) {
                findings.remove(findings.size() - 1);
            }
        }

        private List<String> lines() {
            return findings.stream().map(Finding::detail).toList();
        }
    }

    private record Finding(double severity, String detail) {}

    private static final class ModuleSummary {
        private final DowntimeTracker driveTracker = new DowntimeTracker();
        private final DowntimeTracker turnTracker = new DowntimeTracker();
        private final DowntimeTracker encoderTracker = new DowntimeTracker();
        private final RunningStats activeSpeedRatio = new RunningStats();
        private final RunningStats peerMedianSpeedRatio = new RunningStats();
        private final RunningStats activeSpeedError = new RunningStats();
        private final RunningStats angleErrorDeg = new RunningStats();
        private final RunningStats absRelTurnDeltaDeg = new RunningStats();
        private final RunningStats driveCurrent = new RunningStats();
        private final RunningStats driveVolts = new RunningStats();
        private final RunningStats turnCurrent = new RunningStats();
        private final RunningStats turnVolts = new RunningStats();
        private final RunningStats turnVelocity = new RunningStats();
    }

    private static final class Summary {
        private double lastTimestampSec = 0.0;
        private double lastSampleTimestampSec = Double.NaN;
        private boolean lastSampleEnabled = false;
        private String lastSampleMode = "";
        private double enabledTimeSec = 0.0;
        private double autoTimeSec = 0.0;
        private double teleopTimeSec = 0.0;
        private double disabledTimeSec = 0.0;
        private int totalMeasuredSamples = 0;
        private int enabledMeasuredSamples = 0;
        private int activeTranslationSamples = 0;
        private int activeOmegaSamples = 0;
        private int stationarySamples = 0;
        private final RunningStats sampleDt = new RunningStats();
        private final RunningStats voltageEnabled = new RunningStats();
        private final RunningStats canUtilization = new RunningStats();
        private long maxCanBusOffCount = 0;
        private long maxCanRec = 0;
        private long maxCanTec = 0;
        private long maxCanTxFull = 0;
        private String finalCanStatus = "";
        private final RunningStats sharedSampleCount = new RunningStats();
        private final RunningStats gyroSampleCount = new RunningStats();
        private final RunningStats droppedPhoenixSamples = new RunningStats();
        private final RunningStats droppedTimestampSamples = new RunningStats();
        private final RunningStats requestedLinearSpeed = new RunningStats();
        private final RunningStats setpointLinearSpeed = new RunningStats();
        private final RunningStats measuredLinearSpeed = new RunningStats();
        private final RunningStats translationVectorError = new RunningStats();
        private final RunningStats translationRatio = new RunningStats();
        private final RunningStats omegaError = new RunningStats();
        private final RunningStats omegaRatio = new RunningStats();
        private final RunningStats stationaryCreep = new RunningStats();
        private final ModuleSummary[] moduleSummaries = new ModuleSummary[] {
                new ModuleSummary(), new ModuleSummary(), new ModuleSummary(), new ModuleSummary()
        };
        private final DowntimeTracker gyroTracker = new DowntimeTracker();
        private final DowntimeTracker brownoutTracker = new DowntimeTracker();
        private final DowntimeTracker odomMismatchTracker = new DowntimeTracker();
        private final List<String> connectionEvents = new ArrayList<>();
        private final List<String> canEvents = new ArrayList<>();
        private final List<String> odomEvents = new ArrayList<>();
        private final List<String> stateEvents = new ArrayList<>();
        private final TopFindings lowVoltageFindings = new TopFindings(8);
        private final TopFindings translationLagFindings = new TopFindings(8);
        private final TopFindings rotationLagFindings = new TopFindings(8);
        private final TopFindings stationaryFindings = new TopFindings(8);
        private final TopFindings moduleLagFindings = new TopFindings(12);
        private final TopFindings moduleAngleFindings = new TopFindings(12);
        private final TopFindings balanceFindings = new TopFindings(8);
        private final TopFindings canFindings = new TopFindings(8);
        private final TopFindings odomFindings = new TopFindings(8);

        private void close(double timestampSec) {
            gyroTracker.close(timestampSec);
            brownoutTracker.close(timestampSec);
            odomMismatchTracker.close(timestampSec);
            for (ModuleSummary moduleSummary : moduleSummaries) {
                moduleSummary.driveTracker.close(timestampSec);
                moduleSummary.turnTracker.close(timestampSec);
                moduleSummary.encoderTracker.close(timestampSec);
            }
        }
    }

    private static final class LogReport {
        private final Path wpilog;
        private final String matchLabel;
        private final Summary summary;
        private final Map<Integer, EntryInfo> entries;
        private final EntryIds ids;

        private LogReport(Path wpilog, Summary summary, Map<Integer, EntryInfo> entries, EntryIds ids) {
            this.wpilog = wpilog;
            this.matchLabel = wpilog.getFileName().toString().replace(".wpilog", "");
            this.summary = summary;
            this.entries = entries;
            this.ids = ids;
        }

        private String format() {
            StringBuilder out = new StringBuilder();
            out.append("=== ").append(matchLabel).append(" ===\n");
            out.append("path=").append(wpilog.toAbsolutePath()).append('\n');
            out.append(String.format(Locale.US,
                    "enabled_time=%.2fs auto=%.2fs teleop=%.2fs disabled_seen=%.2fs measured_samples=%d enabled_samples=%d sample_dt=%s\n",
                    summary.enabledTimeSec, summary.autoTimeSec, summary.teleopTimeSec, summary.disabledTimeSec,
                    summary.totalMeasuredSamples, summary.enabledMeasuredSamples, summary.sampleDt.summary("s")));
            out.append(String.format(Locale.US,
                    "battery_enabled=%s battery_low_events=%d brownout_downs=%d brownout_total=%.3fs brownout_longest=%.3fs\n",
                    summary.voltageEnabled.summary("V"),
                    summary.stateEvents.stream().filter(line -> line.contains("batteryLow=")).count(),
                    summary.brownoutTracker.transitionsDown,
                    summary.brownoutTracker.totalDownSec,
                    summary.brownoutTracker.longestDownSec));
            out.append(String.format(Locale.US,
                    "CAN util=%s maxBusOff=%d maxREC=%d maxTEC=%d maxTxFull=%d finalStatus=%s\n",
                    summary.canUtilization.summary(""),
                    summary.maxCanBusOffCount,
                    summary.maxCanRec,
                    summary.maxCanTec,
                    summary.maxCanTxFull,
                    summary.finalCanStatus.isBlank() ? "<none>" : summary.finalCanStatus));
            out.append(String.format(Locale.US,
                    "odometry shared=%s gyro=%s droppedPhoenix=%s droppedTimestamp=%s mismatch_downs=%d mismatch_total=%.3fs longest=%.3fs\n",
                    summary.sharedSampleCount.summary(""),
                    summary.gyroSampleCount.summary(""),
                    summary.droppedPhoenixSamples.summary(""),
                    summary.droppedTimestampSamples.summary(""),
                    summary.odomMismatchTracker.transitionsDown,
                    summary.odomMismatchTracker.totalDownSec,
                    summary.odomMismatchTracker.longestDownSec));
            out.append(String.format(Locale.US,
                    "chassis requested=%s setpoint=%s measured=%s translation_ratio_active=%s vector_err=%s omega_ratio_active=%s omega_err=%s stationary_creep=%s\n",
                    summary.requestedLinearSpeed.summary("m/s"),
                    summary.setpointLinearSpeed.summary("m/s"),
                    summary.measuredLinearSpeed.summary("m/s"),
                    summary.translationRatio.summary(""),
                    summary.translationVectorError.summary("m/s"),
                    summary.omegaRatio.summary(""),
                    summary.omegaError.summary("rad/s"),
                    summary.stationaryCreep.summary("m/s")));
            out.append(String.format(Locale.US,
                    "active_translation_samples=%d active_omega_samples=%d stationary_samples=%d gyro_disconnects=%d gyro_down_total=%.3fs\n",
                    summary.activeTranslationSamples,
                    summary.activeOmegaSamples,
                    summary.stationarySamples,
                    summary.gyroTracker.transitionsDown,
                    summary.gyroTracker.totalDownSec));

            out.append("module summary\n");
            for (int i = 0; i < 4; i++) {
                ModuleSummary module = summary.moduleSummaries[i];
                out.append(String.format(Locale.US,
                        "  %s speedRatio=%s peerRatio=%s speedErr=%s angleErr=%s absRelTurnDelta=%s driveI=%s driveV=%s turnI=%s turnV=%s turnVel=%s disconnects[drive=%d turn=%d enc=%d] downSec[drive=%.3f turn=%.3f enc=%.3f]\n",
                        MODULE_NAMES[i],
                        module.activeSpeedRatio.summary(""),
                        module.peerMedianSpeedRatio.summary(""),
                        module.activeSpeedError.summary("m/s"),
                        module.angleErrorDeg.summary("deg"),
                        module.absRelTurnDeltaDeg.summary("deg"),
                        module.driveCurrent.summary("A"),
                        module.driveVolts.summary("V"),
                        module.turnCurrent.summary("A"),
                        module.turnVolts.summary("V"),
                        module.turnVelocity.summary("rad/s"),
                        module.driveTracker.transitionsDown,
                        module.turnTracker.transitionsDown,
                        module.encoderTracker.transitionsDown,
                        module.driveTracker.totalDownSec,
                        module.turnTracker.totalDownSec,
                        module.encoderTracker.totalDownSec));
            }

            appendSection(out, "top low-voltage findings", summary.lowVoltageFindings.lines());
            appendSection(out, "top translation lag findings", summary.translationLagFindings.lines());
            appendSection(out, "top rotation lag findings", summary.rotationLagFindings.lines());
            appendSection(out, "top module speed lag findings", summary.moduleLagFindings.lines());
            appendSection(out, "top module angle findings", summary.moduleAngleFindings.lines());
            appendSection(out, "top module balance findings", summary.balanceFindings.lines());
            appendSection(out, "top stationary creep findings", summary.stationaryFindings.lines());
            appendSection(out, "top CAN findings", summary.canFindings.lines());
            appendSection(out, "top odometry findings", summary.odomFindings.lines());
            appendSection(out, "connection events", summary.connectionEvents);
            appendSection(out, "CAN events", summary.canEvents);
            appendSection(out, "odometry events", summary.odomEvents);
            appendSection(out, "state events", summary.stateEvents);
            appendSection(out, "mapped entries", mappedEntries());
            return out.toString();
        }

        private long lastCounter(int entryId) {
            return 0L;
        }

        private String lastString(int entryId) {
            EntryInfo info = entries.get(entryId);
            return info == null ? "<missing>" : info.name;
        }

        private List<String> mappedEntries() {
            List<String> lines = new ArrayList<>();
            addMapped(lines, "mode", ids.mode);
            addMapped(lines, "enabled", ids.enabled);
            addMapped(lines, "matchTime", ids.matchTime);
            addMapped(lines, "batteryVoltage", ids.batteryVoltage);
            addMapped(lines, "batteryLow", ids.batteryLow);
            addMapped(lines, "brownout", ids.brownout);
            addMapped(lines, "gyroConnected", ids.gyroConnected);
            addMapped(lines, "canBusUtilization", ids.canBusUtilization);
            addMapped(lines, "canBusOffCount", ids.canBusOffCount);
            addMapped(lines, "canRec", ids.canRec);
            addMapped(lines, "canTec", ids.canTec);
            addMapped(lines, "canTxFull", ids.canTxFull);
            addMapped(lines, "canStatus", ids.canStatus);
            addMapped(lines, "odomModuleSampleCounts", ids.odomModuleSampleCounts);
            addMapped(lines, "odomGyroSampleCount", ids.odomGyroSampleCount);
            addMapped(lines, "odomSharedSampleCount", ids.odomSharedSampleCount);
            addMapped(lines, "odomSampleCountMismatch", ids.odomSampleCountMismatch);
            addMapped(lines, "droppedPhoenixSamples", ids.droppedPhoenixSamples);
            addMapped(lines, "droppedTimestampSamples", ids.droppedTimestampSamples);
            addMapped(lines, "requestedSpeeds", ids.requestedSpeeds);
            addMapped(lines, "setpointSpeeds", ids.setpointSpeeds);
            addMapped(lines, "measuredSpeeds", ids.measuredSpeeds);
            addMapped(lines, "measuredModuleStates", ids.measuredModuleStates);
            addMapped(lines, "optimizedSetpointStates", ids.optimizedSetpointStates);
            addMapped(lines, "pose", ids.pose);
            for (int i = 0; i < 4; i++) {
                addMapped(lines, MODULE_NAMES[i] + " driveConnected", ids.driveConnected[i]);
                addMapped(lines, MODULE_NAMES[i] + " turnConnected", ids.turnConnected[i]);
                addMapped(lines, MODULE_NAMES[i] + " encoderConnected", ids.encoderConnected[i]);
                addMapped(lines, MODULE_NAMES[i] + " driveVelocityRadPerSec", ids.driveVelocityRadPerSec[i]);
                addMapped(lines, MODULE_NAMES[i] + " driveAppliedVolts", ids.driveAppliedVolts[i]);
                addMapped(lines, MODULE_NAMES[i] + " driveCurrentAmps", ids.driveCurrentAmps[i]);
                addMapped(lines, MODULE_NAMES[i] + " turnAbsolutePosition", ids.turnAbsolutePosition[i]);
                addMapped(lines, MODULE_NAMES[i] + " turnPosition", ids.turnPosition[i]);
                addMapped(lines, MODULE_NAMES[i] + " turnAppliedVolts", ids.turnAppliedVolts[i]);
                addMapped(lines, MODULE_NAMES[i] + " turnCurrentAmps", ids.turnCurrentAmps[i]);
            }
            return lines;
        }

        private void addMapped(List<String> out, String label, int entryId) {
            EntryInfo info = entries.get(entryId);
            out.add(label + " = " + (info == null ? "<missing>" : (info.type + " | " + info.name)));
        }
    }

    private static final class CombinedReport {
        private final double totalEnabledTimeSec;
        private final int totalLogs;
        private final int totalGyroDisconnects;
        private final int totalBrownouts;
        private final int totalModuleDisconnects;
        private final double worstVoltage;
        private final double worstTranslationRatioP10;
        private final double worstOmegaRatioP10;
        private final double worstStationaryCreepP99;
        private final double worstAngleErrorP99;
        private final double worstModuleSpeedRatioP10;
        private final double worstAbsRelTurnDeltaP99;
        private final int logsWithAnyConnectionIssue;
        private final int logsWithAnyCanIssue;
        private final int logsWithAnyOdomIssue;

        private CombinedReport(
                double totalEnabledTimeSec,
                int totalLogs,
                int totalGyroDisconnects,
                int totalBrownouts,
                int totalModuleDisconnects,
                double worstVoltage,
                double worstTranslationRatioP10,
                double worstOmegaRatioP10,
                double worstStationaryCreepP99,
                double worstAngleErrorP99,
                double worstModuleSpeedRatioP10,
                double worstAbsRelTurnDeltaP99,
                int logsWithAnyConnectionIssue,
                int logsWithAnyCanIssue,
                int logsWithAnyOdomIssue) {
            this.totalEnabledTimeSec = totalEnabledTimeSec;
            this.totalLogs = totalLogs;
            this.totalGyroDisconnects = totalGyroDisconnects;
            this.totalBrownouts = totalBrownouts;
            this.totalModuleDisconnects = totalModuleDisconnects;
            this.worstVoltage = worstVoltage;
            this.worstTranslationRatioP10 = worstTranslationRatioP10;
            this.worstOmegaRatioP10 = worstOmegaRatioP10;
            this.worstStationaryCreepP99 = worstStationaryCreepP99;
            this.worstAngleErrorP99 = worstAngleErrorP99;
            this.worstModuleSpeedRatioP10 = worstModuleSpeedRatioP10;
            this.worstAbsRelTurnDeltaP99 = worstAbsRelTurnDeltaP99;
            this.logsWithAnyConnectionIssue = logsWithAnyConnectionIssue;
            this.logsWithAnyCanIssue = logsWithAnyCanIssue;
            this.logsWithAnyOdomIssue = logsWithAnyOdomIssue;
        }

        private static CombinedReport from(List<LogReport> reports) {
            double totalEnabledTimeSec = 0.0;
            int totalGyroDisconnects = 0;
            int totalBrownouts = 0;
            int totalModuleDisconnects = 0;
            double worstVoltage = Double.POSITIVE_INFINITY;
            double worstTranslationRatioP10 = Double.POSITIVE_INFINITY;
            double worstOmegaRatioP10 = Double.POSITIVE_INFINITY;
            double worstStationaryCreepP99 = 0.0;
            double worstAngleErrorP99 = 0.0;
            double worstModuleSpeedRatioP10 = Double.POSITIVE_INFINITY;
            double worstAbsRelTurnDeltaP99 = 0.0;
            int logsWithAnyConnectionIssue = 0;
            int logsWithAnyCanIssue = 0;
            int logsWithAnyOdomIssue = 0;

            for (LogReport report : reports) {
                Summary s = report.summary;
                totalEnabledTimeSec += s.enabledTimeSec;
                totalGyroDisconnects += s.gyroTracker.transitionsDown;
                totalBrownouts += s.brownoutTracker.transitionsDown;
                boolean hadConnectionIssue = s.gyroTracker.hadIssue();
                boolean hadModuleDisconnect = false;
                for (ModuleSummary moduleSummary : s.moduleSummaries) {
                    totalModuleDisconnects += moduleSummary.driveTracker.transitionsDown
                            + moduleSummary.turnTracker.transitionsDown
                            + moduleSummary.encoderTracker.transitionsDown;
                    hadModuleDisconnect |= moduleSummary.driveTracker.hadIssue()
                            || moduleSummary.turnTracker.hadIssue()
                            || moduleSummary.encoderTracker.hadIssue();
                    worstAngleErrorP99 = Math.max(worstAngleErrorP99, nanToZero(moduleSummary.angleErrorDeg.percentile(99)));
                    worstAbsRelTurnDeltaP99 = Math.max(worstAbsRelTurnDeltaP99, nanToZero(moduleSummary.absRelTurnDeltaDeg.percentile(99)));
                    worstModuleSpeedRatioP10 = Math.min(worstModuleSpeedRatioP10, nanToOne(moduleSummary.activeSpeedRatio.percentile(10)));
                }
                hadConnectionIssue |= hadModuleDisconnect;
                if (hadConnectionIssue) {
                    logsWithAnyConnectionIssue++;
                }
                if (!s.canEvents.isEmpty() || !s.canFindings.lines().isEmpty()) {
                    logsWithAnyCanIssue++;
                }
                if (s.odomMismatchTracker.hadIssue() || !s.odomEvents.isEmpty() || !s.odomFindings.lines().isEmpty()) {
                    logsWithAnyOdomIssue++;
                }
                worstVoltage = Math.min(worstVoltage, nanToInfinity(s.voltageEnabled.min()));
                worstTranslationRatioP10 = Math.min(worstTranslationRatioP10, nanToOne(s.translationRatio.percentile(10)));
                worstOmegaRatioP10 = Math.min(worstOmegaRatioP10, nanToOne(s.omegaRatio.percentile(10)));
                worstStationaryCreepP99 = Math.max(worstStationaryCreepP99, nanToZero(s.stationaryCreep.percentile(99)));
            }

            return new CombinedReport(
                    totalEnabledTimeSec,
                    reports.size(),
                    totalGyroDisconnects,
                    totalBrownouts,
                    totalModuleDisconnects,
                    worstVoltage,
                    worstTranslationRatioP10,
                    worstOmegaRatioP10,
                    worstStationaryCreepP99,
                    worstAngleErrorP99,
                    worstModuleSpeedRatioP10,
                    worstAbsRelTurnDeltaP99,
                    logsWithAnyConnectionIssue,
                    logsWithAnyCanIssue,
                    logsWithAnyOdomIssue);
        }

        private String summary() {
            return String.format(Locale.US,
                    "combined_summary\n  total_enabled_time=%.1fs across %d logs\n  disconnects gyro=%d moduleSignals=%d brownouts=%d logsWithConnectionIssue=%d/%d logsWithCanIssue=%d/%d logsWithOdomIssue=%d/%d\n  worst_voltage=%.2fV worst_translation_ratio_p10=%.2f worst_omega_ratio_p10=%.2f worst_stationary_creep_p99=%.2f m/s\n  worst_module_speed_ratio_p10=%.2f worst_module_angle_error_p99=%.1f deg worst_abs_rel_turn_delta_p99=%.1f deg\n",
                    totalEnabledTimeSec,
                    totalLogs,
                    totalGyroDisconnects,
                    totalModuleDisconnects,
                    totalBrownouts,
                    logsWithAnyConnectionIssue,
                    totalLogs,
                    logsWithAnyCanIssue,
                    totalLogs,
                    logsWithAnyOdomIssue,
                    totalLogs,
                    worstVoltage,
                    worstTranslationRatioP10,
                    worstOmegaRatioP10,
                    worstStationaryCreepP99,
                    worstModuleSpeedRatioP10,
                    worstAngleErrorP99,
                    worstAbsRelTurnDeltaP99);
        }

        private static double nanToZero(double value) {
            return Double.isFinite(value) ? value : 0.0;
        }

        private static double nanToOne(double value) {
            return Double.isFinite(value) ? value : 1.0;
        }

        private static double nanToInfinity(double value) {
            return Double.isFinite(value) ? value : Double.POSITIVE_INFINITY;
        }
    }

    private static void appendSection(StringBuilder out, String title, List<String> lines) {
        out.append(title).append('\n');
        if (lines.isEmpty()) {
            out.append("  <none>\n");
            return;
        }
        for (String line : lines) {
            out.append("  ").append(line).append('\n');
        }
    }
}
