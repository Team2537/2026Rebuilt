package frc.robot.autos;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.util.struct.StructBuffer;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.junit.jupiter.api.Test;

class MdBetAutonomousLogAnalyzerTest {
    private static final ObjectMapper OBJECT_MAPPER = new ObjectMapper();
    private static final Pattern QUAL_FILE_PATTERN = Pattern.compile(".*_q(\\d+)\\.wpilog$");
    private static final Pattern AUTO_RESET_ODOM_PATTERN = Pattern.compile("\\\"resetOdom\\\"\\s*:\\s*(true|false)");
    private static final double TRAJECTORY_SETPOINT_FRESHNESS_SEC = 0.10;
    private static final double HIGH_REQUEST_SPEED_THRESHOLD_MPS = 1.0;
    private static final double SIGNIFICANT_TRANSFER_PERCENT = 0.05;
    private static final double SIGNIFICANT_KICKER_TORQUE_AMPS = 1.0;

    @Test
    void analyzeMdbetAutonomousLogsAfterQual7() throws Exception {
        Path logDir = Path.of("logs/mdbet").toAbsolutePath();
        assertTrue(Files.isDirectory(logDir), "Missing log directory: " + logDir);

        List<Path> logs;
        try (var stream = Files.list(logDir)) {
            logs = stream.filter(Files::isRegularFile)
                    .filter(path -> matchNumber(path) > 7)
                    .sorted()
                    .toList();
        }
        assertTrue(!logs.isEmpty(), "Expected MDBET logs after qual 7 in " + logDir);

        List<LogAnalysis> analyses = new ArrayList<>();
        for (Path log : logs) {
            analyses.add(analyzeLog(log));
        }

        String report = formatReport(analyses);
        Path out = Path.of("build/reports/mdbet-autonomous-log-analysis.txt").toAbsolutePath();
        Files.createDirectories(out.getParent());
        Files.writeString(out, report);

        System.out.println(report);
        System.out.println("MDBET autonomous analysis written: " + out);
    }

    private static LogAnalysis analyzeLog(Path wpilog) throws IOException {
        Map<Integer, EntryInfo> entries = readEntries(wpilog);
        EntryIds ids = EntryIds.from(entries);

        DataLogReader reader = new DataLogReader(wpilog.toString());
        if (!reader.isValid()) {
            throw new IllegalStateException("Invalid WPILOG file: " + wpilog);
        }

        StructBuffer<Pose2d> poseBuffer = StructBuffer.create(Pose2d.struct);
        StructBuffer<ChassisSpeeds> speedsBuffer = StructBuffer.create(ChassisSpeeds.struct);

        boolean enabled = false;
        String mode = "";
        double matchTime = Double.NaN;
        String selectedRoutine = "";
        String selectedCommand = "";
        String lastCommandEvent = "";

        Pose2d robotPose = new Pose2d();
        Pose2d trajectorySetpoint = new Pose2d();
        long trajectorySetpointTimestampUs = Long.MIN_VALUE;
        ChassisSpeeds requestedSpeeds = new ChassisSpeeds();
        ChassisSpeeds measuredSpeeds = new ChassisSpeeds();

        boolean brownout = false;
        boolean batteryLow = false;
        double batteryVoltage = Double.NaN;
        boolean gyroConnected = true;
        boolean visionConnected = true;
        boolean cam0Connected = true;
        boolean cam1Connected = true;
        boolean odometryMismatch = false;
        double canBusUtilization = Double.NaN;
        double canTxFullCount = Double.NaN;
        int visionRejectCount = 0;
        int visionJumpCount = 0;

        boolean automaticFeedEnabled = false;
        boolean shooterAtSetpoint = false;
        boolean aimReady = false;
        boolean gateOpen = false;
        String blockReason = "";
        String shootingState = "";
        double aimErrorDeg = Double.NaN;
        boolean pathRotationOverrideEnabled = false;
        double kickerTorqueAmps = 0.0;
        double transferCommandedPercent = 0.0;
        boolean intakeExtended = false;

        AutonomousWindow activeWindow = null;
        List<AutonomousWindow> windows = new ArrayList<>();

        Iterator<DataLogRecord> iterator = reader.iterator();
        while (true) {
            final DataLogRecord record;
            try {
                if (!iterator.hasNext()) {
                    break;
                }
                record = iterator.next();
            } catch (IllegalArgumentException ignored) {
                break;
            }

            if (record.isStart() || record.isControl()) {
                continue;
            }

            long ts = record.getTimestamp();
            int entry = record.getEntry();
            boolean commandEventChanged = false;

            if (entry == ids.enabledEntry) {
                enabled = readBooleanLenient(record, entries.get(entry).name());
            } else if (entry == ids.modeEntry) {
                mode = readStringLenient(record);
            } else if (entry == ids.matchTimeEntry) {
                matchTime = readDoubleLenient(record);
            } else if (entry == ids.selectedRoutineEntry) {
                selectedRoutine = readStringLenient(record);
            } else if (entry == ids.selectedCommandEntry) {
                selectedCommand = readStringLenient(record);
            } else if (entry == ids.commandLastEventEntry) {
                String newValue = readStringLenient(record);
                if (!newValue.equals(lastCommandEvent)) {
                    lastCommandEvent = newValue;
                    commandEventChanged = true;
                }
            } else if (entry == ids.poseEntry) {
                robotPose = poseBuffer.read(record.getRaw());
            } else if (entry == ids.trajectorySetpointEntry) {
                trajectorySetpoint = poseBuffer.read(record.getRaw());
                trajectorySetpointTimestampUs = ts;
            } else if (entry == ids.requestedSpeedsEntry) {
                requestedSpeeds = speedsBuffer.read(record.getRaw());
            } else if (entry == ids.measuredSpeedsEntry) {
                measuredSpeeds = speedsBuffer.read(record.getRaw());
            } else if (entry == ids.brownoutEntry) {
                brownout = readBooleanLenient(record, entries.get(entry).name());
            } else if (entry == ids.batteryLowEntry) {
                batteryLow = readBooleanLenient(record, entries.get(entry).name());
            } else if (entry == ids.batteryVoltageEntry) {
                batteryVoltage = readDoubleLenient(record);
            } else if (entry == ids.gyroConnectedEntry) {
                gyroConnected = readBooleanLenient(record, entries.get(entry).name());
            } else if (entry == ids.visionConnectedEntry) {
                visionConnected = readBooleanLenient(record, entries.get(entry).name());
            } else if (entry == ids.cam0ConnectedEntry) {
                cam0Connected = readBooleanLenient(record, entries.get(entry).name());
            } else if (entry == ids.cam1ConnectedEntry) {
                cam1Connected = readBooleanLenient(record, entries.get(entry).name());
            } else if (entry == ids.odometryMismatchEntry) {
                odometryMismatch = readBooleanLenient(record, entries.get(entry).name());
            } else if (entry == ids.canBusUtilizationEntry) {
                canBusUtilization = readDoubleLenient(record);
            } else if (entry == ids.canTxFullCountEntry) {
                canTxFullCount = readDoubleLenient(record);
            } else if (entry == ids.visionRejectCountEntry) {
                visionRejectCount = (int) Math.round(readDoubleLenient(record));
            } else if (entry == ids.visionJumpCountEntry) {
                visionJumpCount = (int) Math.round(readDoubleLenient(record));
            } else if (entry == ids.automaticFeedEnabledEntry) {
                automaticFeedEnabled = readBooleanLenient(record, entries.get(entry).name());
            } else if (entry == ids.shooterAtSetpointEntry) {
                shooterAtSetpoint = readBooleanLenient(record, entries.get(entry).name());
            } else if (entry == ids.aimReadyEntry) {
                aimReady = readBooleanLenient(record, entries.get(entry).name());
            } else if (entry == ids.gateOpenEntry) {
                gateOpen = readBooleanLenient(record, entries.get(entry).name());
            } else if (entry == ids.blockReasonEntry) {
                blockReason = readStringLenient(record);
            } else if (entry == ids.shootingStateEntry) {
                shootingState = readStringLenient(record);
            } else if (entry == ids.aimErrorDegEntry) {
                aimErrorDeg = readDoubleLenient(record);
            } else if (entry == ids.pathRotationOverrideEnabledEntry) {
                pathRotationOverrideEnabled = readBooleanLenient(record, entries.get(entry).name());
            } else if (entry == ids.kickerTorqueAmpsEntry) {
                kickerTorqueAmps = readDoubleLenient(record);
            } else if (entry == ids.transferCommandedPercentEntry) {
                transferCommandedPercent = readDoubleLenient(record);
            } else if (entry == ids.intakeExtendedEntry) {
                intakeExtended = readBooleanLenient(record, entries.get(entry).name());
            }

            boolean autoActiveNow = enabled && isAutonomousMode(mode);
            if (activeWindow == null && autoActiveNow) {
                activeWindow = new AutonomousWindow(
                        ts,
                        matchTime,
                        selectedRoutine,
                        selectedCommand,
                        loadAutoExpectation(selectedRoutine, selectedCommand),
                        visionRejectCount,
                        visionJumpCount,
                        canTxFullCount);
            }

            if (activeWindow != null && autoActiveNow) {
                activeWindow.observeHealth(
                        ts,
                        matchTime,
                        batteryVoltage,
                        batteryLow,
                        brownout,
                        gyroConnected,
                        visionConnected,
                        cam0Connected,
                        cam1Connected,
                        odometryMismatch,
                        canBusUtilization,
                        canTxFullCount,
                        visionRejectCount,
                        visionJumpCount);

                if (commandEventChanged) {
                    activeWindow.observeCommandEvent(ts, lastCommandEvent);
                }
                if (entry == ids.poseEntry) {
                    activeWindow.observePoseSample(
                            ts,
                            matchTime,
                            robotPose,
                            trajectorySetpoint,
                            trajectorySetpointTimestampUs,
                            requestedSpeeds,
                            measuredSpeeds,
                            automaticFeedEnabled,
                            shooterAtSetpoint,
                            aimReady,
                            gateOpen,
                            blockReason,
                            shootingState,
                            aimErrorDeg,
                            pathRotationOverrideEnabled,
                            kickerTorqueAmps,
                            transferCommandedPercent,
                            intakeExtended,
                            batteryLow);
                }
            }

            if (activeWindow != null && !autoActiveNow) {
                activeWindow.finish(ts, matchTime, robotPose, measuredSpeeds, selectedRoutine, selectedCommand);
                windows.add(activeWindow);
                activeWindow = null;
            }
        }

        if (activeWindow != null) {
            activeWindow.finish(Long.MIN_VALUE, matchTime, robotPose, measuredSpeeds, selectedRoutine, selectedCommand);
            windows.add(activeWindow);
        }

        return new LogAnalysis(wpilog, entries, ids, windows);
    }

    private static Map<Integer, EntryInfo> readEntries(Path wpilog) throws IOException {
        DataLogReader reader = new DataLogReader(wpilog.toString());
        if (!reader.isValid()) {
            throw new IllegalStateException("Invalid WPILOG file: " + wpilog);
        }
        Map<Integer, EntryInfo> entries = new HashMap<>();
        Iterator<DataLogRecord> iterator = reader.iterator();
        while (true) {
            final DataLogRecord record;
            try {
                if (!iterator.hasNext()) {
                    break;
                }
                record = iterator.next();
            } catch (IllegalArgumentException ignored) {
                break;
            }
            if (!record.isStart()) {
                continue;
            }
            var start = record.getStartData();
            entries.put(start.entry, new EntryInfo(start.name, start.type));
        }
        return entries;
    }

    private static AutoExpectation loadAutoExpectation(String selectedRoutine, String selectedCommand) throws IOException {
        String autoName = "";
        if (selectedRoutine != null && selectedRoutine.startsWith("pp/")) {
            autoName = selectedRoutine.substring("pp/".length());
        } else if (selectedCommand != null && selectedCommand.startsWith("AutoPathPlanner_")) {
            autoName = selectedCommand.substring("AutoPathPlanner_".length());
        }
        if (autoName.isBlank()) {
            return new AutoExpectation("", false, List.of(), Set.of(), Set.of(), null);
        }

        Path autoFile = Path.of("src/main/deploy/pathplanner/autos", autoName + ".auto").toAbsolutePath();
        if (!Files.exists(autoFile)) {
            return new AutoExpectation(autoName, false, List.of(), Set.of(), Set.of(), autoFile);
        }

        String json = Files.readString(autoFile);
        JsonNode autoRoot = OBJECT_MAPPER.readTree(json);
        List<String> pathNames = new ArrayList<>();
        collectPathNames(autoRoot, pathNames);
        Set<String> topLevelNamedCommands = new LinkedHashSet<>();
        collectNamedCommands(autoRoot.path("command"), topLevelNamedCommands);
        Set<String> pathEventNamedCommands = new LinkedHashSet<>();
        for (String pathName : pathNames) {
            Path pathFile = Path.of("src/main/deploy/pathplanner/paths", pathName + ".path").toAbsolutePath();
            if (!Files.exists(pathFile)) {
                continue;
            }
            JsonNode pathRoot = OBJECT_MAPPER.readTree(Files.readString(pathFile));
            for (JsonNode marker : pathRoot.path("eventMarkers")) {
                collectNamedCommands(marker.path("command"), pathEventNamedCommands);
            }
        }
        boolean resetOdom = false;
        Matcher resetMatcher = AUTO_RESET_ODOM_PATTERN.matcher(json);
        if (resetMatcher.find()) {
            resetOdom = Boolean.parseBoolean(resetMatcher.group(1));
        }
        return new AutoExpectation(autoName, resetOdom, pathNames, topLevelNamedCommands, pathEventNamedCommands, autoFile);
    }

    private static void collectPathNames(JsonNode node, List<String> pathNames) {
        if (node == null || node.isMissingNode() || node.isNull()) {
            return;
        }
        if (node.isObject()) {
            JsonNode pathNameNode = node.get("pathName");
            if (pathNameNode != null && pathNameNode.isTextual()) {
                pathNames.add(pathNameNode.asText());
            }
            node.fields().forEachRemaining(entry -> collectPathNames(entry.getValue(), pathNames));
            return;
        }
        if (node.isArray()) {
            for (JsonNode child : node) {
                collectPathNames(child, pathNames);
            }
        }
    }

    private static void collectNamedCommands(JsonNode node, Set<String> namedCommands) {
        if (node == null || node.isMissingNode() || node.isNull()) {
            return;
        }
        if (node.isObject()) {
            JsonNode typeNode = node.get("type");
            JsonNode dataNode = node.get("data");
            if (typeNode != null
                    && typeNode.isTextual()
                    && "named".equals(typeNode.asText())
                    && dataNode != null
                    && dataNode.get("name") != null
                    && dataNode.get("name").isTextual()) {
                namedCommands.add(dataNode.get("name").asText());
            }
            node.fields().forEachRemaining(entry -> collectNamedCommands(entry.getValue(), namedCommands));
            return;
        }
        if (node.isArray()) {
            for (JsonNode child : node) {
                collectNamedCommands(child, namedCommands);
            }
        }
    }

    private static int matchNumber(Path path) {
        Matcher matcher = QUAL_FILE_PATTERN.matcher(path.getFileName().toString());
        return matcher.matches() ? Integer.parseInt(matcher.group(1)) : -1;
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

    private static double readDoubleLenient(DataLogRecord record) {
        try {
            return record.getDouble();
        } catch (Exception ignored) {
        }
        try {
            return record.getInteger();
        } catch (Exception ignored) {
        }
        return Double.NaN;
    }

    private static String readStringLenient(DataLogRecord record) {
        try {
            return record.getString();
        } catch (Exception ignored) {
        }
        return "";
    }

    private static boolean isAutonomousMode(String mode) {
        if (mode == null) {
            return false;
        }
        String normalized = mode.trim().toUpperCase(Locale.ROOT);
        return normalized.equals("AUTO") || normalized.equals("AUTONOMOUS");
    }

    private static String formatReport(List<LogAnalysis> analyses) {
        StringBuilder out = new StringBuilder();
        out.append("MDBET autonomous log analysis (quals after q7)\n");
        out.append("logs_analyzed=").append(analyses.size()).append('\n');
        out.append('\n');

        for (LogAnalysis analysis : analyses) {
            out.append("================================================================================\n");
            out.append("log=").append(analysis.wpilog().toAbsolutePath()).append('\n');
            out.append("match=").append(matchNumber(analysis.wpilog())).append('\n');
            out.append("mapped_entries\n");
            out.append(analysis.entryIds().formatMappedEntries()).append('\n');

            if (analysis.windows().isEmpty()) {
                out.append("autonomous_windows\n  <none>\n\n");
                continue;
            }

            out.append("autonomous_windows=").append(analysis.windows().size()).append('\n');
            for (int i = 0; i < analysis.windows().size(); i++) {
                AutonomousWindow window = analysis.windows().get(i);
                out.append("\nwindow[").append(i).append("]\n");
                out.append(window.format());
            }
            out.append('\n');
        }

        out.append("================================================================================\n");
        out.append("aggregate_summary\n");
        int totalWindows = analyses.stream().mapToInt(a -> a.windows().size()).sum();
        int issueWindows = 0;
        Distribution aggregatePathP95 = new Distribution();
        Distribution aggregateFinalPath = new Distribution();
        Distribution aggregateAimGate = new Distribution();
        for (LogAnalysis analysis : analyses) {
            for (AutonomousWindow window : analysis.windows()) {
                if (!window.issues.isEmpty()) {
                    issueWindows++;
                }
                if (window.pathTranslationErrors.count() > 0) {
                    aggregatePathP95.add(window.pathTranslationErrors.percentile(95.0));
                    aggregateFinalPath.add(window.lastPathErrorMeters);
                }
                if (window.aimErrorsWhileGateOpenDeg.count() > 0) {
                    aggregateAimGate.add(window.aimErrorsWhileGateOpenDeg.percentile(95.0));
                }
            }
        }
        out.append("  total_windows=").append(totalWindows).append('\n');
        out.append("  windows_with_issues=").append(issueWindows).append('\n');
        out.append("  path_error_p95_across_windows=").append(aggregatePathP95.summaryMeters()).append('\n');
        out.append("  final_path_error_across_windows=").append(aggregateFinalPath.summaryMeters()).append('\n');
        out.append("  gate_open_aim_error_p95_across_windows=").append(aggregateAimGate.summaryDegrees()).append('\n');
        return out.toString();
    }

    private record LogAnalysis(
            Path wpilog,
            Map<Integer, EntryInfo> entries,
            EntryIds entryIds,
            List<AutonomousWindow> windows) {
    }

    private record EntryInfo(String name, String type) {
    }

    private record AutoExpectation(
            String autoName,
            boolean resetOdom,
            List<String> pathNames,
            Set<String> topLevelNamedCommands,
            Set<String> pathEventNamedCommands,
            Path autoFile) {
        private boolean isPathPlannerAuto() {
            return autoName != null && !autoName.isBlank();
        }

        private Set<String> allNamedCommands() {
            Set<String> out = new LinkedHashSet<>();
            out.addAll(pathEventNamedCommands);
            out.addAll(topLevelNamedCommands);
            return out;
        }
    }

    private static final class EntryIds {
        private final int enabledEntry;
        private final int modeEntry;
        private final int matchTimeEntry;
        private final int selectedRoutineEntry;
        private final int selectedCommandEntry;
        private final int commandLastEventEntry;
        private final int poseEntry;
        private final int trajectorySetpointEntry;
        private final int requestedSpeedsEntry;
        private final int measuredSpeedsEntry;
        private final int brownoutEntry;
        private final int batteryLowEntry;
        private final int batteryVoltageEntry;
        private final int gyroConnectedEntry;
        private final int visionConnectedEntry;
        private final int cam0ConnectedEntry;
        private final int cam1ConnectedEntry;
        private final int odometryMismatchEntry;
        private final int canBusUtilizationEntry;
        private final int canTxFullCountEntry;
        private final int visionRejectCountEntry;
        private final int visionJumpCountEntry;
        private final int automaticFeedEnabledEntry;
        private final int shooterAtSetpointEntry;
        private final int aimReadyEntry;
        private final int gateOpenEntry;
        private final int blockReasonEntry;
        private final int shootingStateEntry;
        private final int aimErrorDegEntry;
        private final int pathRotationOverrideEnabledEntry;
        private final int kickerTorqueAmpsEntry;
        private final int transferCommandedPercentEntry;
        private final int intakeExtendedEntry;

        private EntryIds(
                int enabledEntry,
                int modeEntry,
                int matchTimeEntry,
                int selectedRoutineEntry,
                int selectedCommandEntry,
                int commandLastEventEntry,
                int poseEntry,
                int trajectorySetpointEntry,
                int requestedSpeedsEntry,
                int measuredSpeedsEntry,
                int brownoutEntry,
                int batteryLowEntry,
                int batteryVoltageEntry,
                int gyroConnectedEntry,
                int visionConnectedEntry,
                int cam0ConnectedEntry,
                int cam1ConnectedEntry,
                int odometryMismatchEntry,
                int canBusUtilizationEntry,
                int canTxFullCountEntry,
                int visionRejectCountEntry,
                int visionJumpCountEntry,
                int automaticFeedEnabledEntry,
                int shooterAtSetpointEntry,
                int aimReadyEntry,
                int gateOpenEntry,
                int blockReasonEntry,
                int shootingStateEntry,
                int aimErrorDegEntry,
                int pathRotationOverrideEnabledEntry,
                int kickerTorqueAmpsEntry,
                int transferCommandedPercentEntry,
                int intakeExtendedEntry) {
            this.enabledEntry = enabledEntry;
            this.modeEntry = modeEntry;
            this.matchTimeEntry = matchTimeEntry;
            this.selectedRoutineEntry = selectedRoutineEntry;
            this.selectedCommandEntry = selectedCommandEntry;
            this.commandLastEventEntry = commandLastEventEntry;
            this.poseEntry = poseEntry;
            this.trajectorySetpointEntry = trajectorySetpointEntry;
            this.requestedSpeedsEntry = requestedSpeedsEntry;
            this.measuredSpeedsEntry = measuredSpeedsEntry;
            this.brownoutEntry = brownoutEntry;
            this.batteryLowEntry = batteryLowEntry;
            this.batteryVoltageEntry = batteryVoltageEntry;
            this.gyroConnectedEntry = gyroConnectedEntry;
            this.visionConnectedEntry = visionConnectedEntry;
            this.cam0ConnectedEntry = cam0ConnectedEntry;
            this.cam1ConnectedEntry = cam1ConnectedEntry;
            this.odometryMismatchEntry = odometryMismatchEntry;
            this.canBusUtilizationEntry = canBusUtilizationEntry;
            this.canTxFullCountEntry = canTxFullCountEntry;
            this.visionRejectCountEntry = visionRejectCountEntry;
            this.visionJumpCountEntry = visionJumpCountEntry;
            this.automaticFeedEnabledEntry = automaticFeedEnabledEntry;
            this.shooterAtSetpointEntry = shooterAtSetpointEntry;
            this.aimReadyEntry = aimReadyEntry;
            this.gateOpenEntry = gateOpenEntry;
            this.blockReasonEntry = blockReasonEntry;
            this.shootingStateEntry = shootingStateEntry;
            this.aimErrorDegEntry = aimErrorDegEntry;
            this.pathRotationOverrideEnabledEntry = pathRotationOverrideEnabledEntry;
            this.kickerTorqueAmpsEntry = kickerTorqueAmpsEntry;
            this.transferCommandedPercentEntry = transferCommandedPercentEntry;
            this.intakeExtendedEntry = intakeExtendedEntry;
        }

        private static EntryIds from(Map<Integer, EntryInfo> entries) {
            return new EntryIds(
                    findEntry(entries, "RobotState/Enabled"),
                    findEntry(entries, "RobotState/Mode"),
                    findEntry(entries, "RobotState/MatchTime"),
                    findEntry(entries, "Auto/selector/selectedRoutineName"),
                    findEntry(entries, "Auto/selector/selectedCommandName"),
                    findEntry(entries, "Commands/lastEvent"),
                    findEntry(entries, "Odometry/Robot"),
                    findEntry(entries, "Odometry/TrajectorySetpoint"),
                    findEntry(entries, "SwerveChassisSpeeds/Requested"),
                    findEntry(entries, "SwerveChassisSpeeds/Measured"),
                    findEntry(entries, "SystemStats/BrownedOut"),
                    findEntry(entries, "RobotState/BatteryLow"),
                    findEntry(entries, "RobotState/BatteryVoltage", "PowerDistribution/Voltage", "PowerDistribution/totalVoltageVolts"),
                    findEntry(entries, "RobotState/GyroConnected"),
                    findEntry(entries, "RobotState/VisionConnected"),
                    findEntry(entries, "RobotState/Vision/Camera0Connected"),
                    findEntry(entries, "RobotState/Vision/Camera1Connected"),
                    findEntry(entries, "Drive/Odometry/SampleCountMismatch"),
                    findEntry(entries, "CAN/CANivore/BusUtilization"),
                    findEntry(entries, "CAN/CANivore/TxFullCount"),
                    findEntry(entries, "Vision/events/rejectCount"),
                    findEntry(entries, "Vision/events/jumpCount"),
                    findEntry(entries, "Shooting/AutomaticFeedEnabled"),
                    findEntry(entries, "Shooting/ShooterAtSetpoint"),
                    findEntry(entries, "Shooting/AimReady"),
                    findEntry(entries, "Shooting/GateOpen"),
                    findEntry(entries, "Shooting/BlockReason"),
                    findEntry(entries, "Shooting/State"),
                    findEntry(entries, "Shooting/AimErrorDeg"),
                    findEntry(entries, "AutoAim/PathRotationOverrideEnabled"),
                    findEntry(entries, "Shooter/KickerTorqueAmps"),
                    findEntry(entries, "Transfer/CommandedPercent"),
                    findEntry(entries, "Intake/Extended"));
        }

        private static int findEntry(Map<Integer, EntryInfo> entries, String... suffixes) {
            for (String suffix : suffixes) {
                for (var entry : entries.entrySet()) {
                    if (entry.getValue().name().endsWith(suffix)) {
                        return entry.getKey();
                    }
                }
            }
            return -1;
        }

        private String formatMappedEntries() {
            return String.format(
                    Locale.US,
                    "  enabled=%d%n"
                            + "  mode=%d%n"
                            + "  matchTime=%d%n"
                            + "  selectedRoutine=%d%n"
                            + "  selectedCommand=%d%n"
                            + "  commandLastEvent=%d%n"
                            + "  pose=%d%n"
                            + "  trajectorySetpoint=%d%n"
                            + "  requestedSpeeds=%d%n"
                            + "  measuredSpeeds=%d%n"
                            + "  batteryVoltage=%d%n"
                            + "  batteryLow=%d%n"
                            + "  brownout=%d%n"
                            + "  gyroConnected=%d%n"
                            + "  visionConnected=%d%n"
                            + "  cam0Connected=%d%n"
                            + "  cam1Connected=%d%n"
                            + "  odometryMismatch=%d%n"
                            + "  canBusUtilization=%d%n"
                            + "  canTxFullCount=%d%n"
                            + "  visionRejectCount=%d%n"
                            + "  visionJumpCount=%d%n"
                            + "  automaticFeedEnabled=%d%n"
                            + "  shooterAtSetpoint=%d%n"
                            + "  aimReady=%d%n"
                            + "  gateOpen=%d%n"
                            + "  blockReason=%d%n"
                            + "  shootingState=%d%n"
                            + "  aimErrorDeg=%d%n"
                            + "  pathRotationOverrideEnabled=%d%n"
                            + "  kickerTorqueAmps=%d%n"
                            + "  transferCommandedPercent=%d%n"
                            + "  intakeExtended=%d%n",
                    enabledEntry,
                    modeEntry,
                    matchTimeEntry,
                    selectedRoutineEntry,
                    selectedCommandEntry,
                    commandLastEventEntry,
                    poseEntry,
                    trajectorySetpointEntry,
                    requestedSpeedsEntry,
                    measuredSpeedsEntry,
                    batteryVoltageEntry,
                    batteryLowEntry,
                    brownoutEntry,
                    gyroConnectedEntry,
                    visionConnectedEntry,
                    cam0ConnectedEntry,
                    cam1ConnectedEntry,
                    odometryMismatchEntry,
                    canBusUtilizationEntry,
                    canTxFullCountEntry,
                    visionRejectCountEntry,
                    visionJumpCountEntry,
                    automaticFeedEnabledEntry,
                    shooterAtSetpointEntry,
                    aimReadyEntry,
                    gateOpenEntry,
                    blockReasonEntry,
                    shootingStateEntry,
                    aimErrorDegEntry,
                    pathRotationOverrideEnabledEntry,
                    kickerTorqueAmpsEntry,
                    transferCommandedPercentEntry,
                    intakeExtendedEntry);
        }
    }

    private static final class AutonomousWindow {
        private final long startUs;
        private long endUs = Long.MIN_VALUE;
        private final double startMatchTime;
        private final String initialSelectedRoutine;
        private final String initialSelectedCommand;
        private final AutoExpectation autoExpectation;
        private String finalSelectedRoutine;
        private String finalSelectedCommand;
        private Pose2d finalRobotPose = new Pose2d();
        private ChassisSpeeds finalMeasuredSpeeds = new ChassisSpeeds();

        private final List<String> issues = new ArrayList<>();
        private final List<String> commandEvents = new ArrayList<>();
        private final List<String> healthEvents = new ArrayList<>();
        private final Set<String> startedCommandNames = new LinkedHashSet<>();
        private final Map<String, Integer> blockReasonCounts = new HashMap<>();

        private final Distribution pathTranslationErrors = new Distribution();
        private final Distribution pathHeadingErrorsDeg = new Distribution();
        private final Distribution pathSpeedErrorsMps = new Distribution();
        private final Distribution highRequestResponseRatios = new Distribution();
        private final Distribution aimErrorsWhileAutoFeedDeg = new Distribution();
        private final Distribution aimErrorsWhileGateOpenDeg = new Distribution();

        private long poseSamples = 0;
        private long pathTrackedPoseSamples = 0;
        private long autoFeedPoseSamples = 0;
        private long shooterAtPoseSamples = 0;
        private long aimReadyPoseSamples = 0;
        private long gateOpenPoseSamples = 0;
        private long pathRotationOverridePoseSamples = 0;
        private long transferActivePoseSamples = 0;
        private long kickerActivePoseSamples = 0;
        private long intakeExtendedPoseSamples = 0;
        private long brownoutPoseSamples = 0;
        private long batteryLowPoseSamples = 0;
        private long gyroDisconnectedPoseSamples = 0;
        private long visionDisconnectedPoseSamples = 0;
        private long cam0DisconnectedPoseSamples = 0;
        private long cam1DisconnectedPoseSamples = 0;
        private long odometryMismatchPoseSamples = 0;

        private double minBatteryVoltage = Double.POSITIVE_INFINITY;
        private double maxCanBusUtilization = Double.NaN;
        private double maxCanTxFullCount = Double.NaN;
        private int visionRejectCountStart;
        private int visionRejectCountEnd;
        private int visionJumpCountStart;
        private int visionJumpCountEnd;
        private double canTxFullCountStart;
        private double canTxFullCountEnd;

        private long firstPathSampleUs = Long.MIN_VALUE;
        private double firstPathErrorMeters = Double.NaN;
        private double firstPathHeadingErrorDeg = Double.NaN;
        private long lastPathSampleUs = Long.MIN_VALUE;
        private double lastPathErrorMeters = Double.NaN;
        private double lastPathHeadingErrorDeg = Double.NaN;
        private long firstAutoFeedUs = Long.MIN_VALUE;
        private long firstGateOpenUs = Long.MIN_VALUE;

        private long firstSelectedAutoStartUs = Long.MIN_VALUE;
        private int selectedAutoStarts = 0;
        private int selectedAutoFinishes = 0;
        private int selectedAutoInterrupts = 0;

        private boolean lastBrownoutState = false;
        private boolean lastBatteryLowState = false;
        private boolean lastGyroConnectedState = true;
        private boolean lastVisionConnectedState = true;
        private boolean lastCam0ConnectedState = true;
        private boolean lastCam1ConnectedState = true;
        private boolean lastOdometryMismatchState = false;

        private Pose2d previousRobotPose = null;
        private double drivenDistanceMeters = 0.0;

        private AutonomousWindow(
                long startUs,
                double startMatchTime,
                String selectedRoutine,
                String selectedCommand,
                AutoExpectation autoExpectation,
                int visionRejectCountStart,
                int visionJumpCountStart,
                double canTxFullCountStart) {
            this.startUs = startUs;
            this.startMatchTime = startMatchTime;
            this.initialSelectedRoutine = selectedRoutine;
            this.initialSelectedCommand = selectedCommand;
            this.autoExpectation = autoExpectation;
            this.finalSelectedRoutine = selectedRoutine;
            this.finalSelectedCommand = selectedCommand;
            this.visionRejectCountStart = visionRejectCountStart;
            this.visionRejectCountEnd = visionRejectCountStart;
            this.visionJumpCountStart = visionJumpCountStart;
            this.visionJumpCountEnd = visionJumpCountStart;
            this.canTxFullCountStart = canTxFullCountStart;
            this.canTxFullCountEnd = canTxFullCountStart;
        }

        private void observeHealth(
                long ts,
                double matchTime,
                double batteryVoltage,
                boolean batteryLow,
                boolean brownout,
                boolean gyroConnected,
                boolean visionConnected,
                boolean cam0Connected,
                boolean cam1Connected,
                boolean odometryMismatch,
                double canBusUtilization,
                double canTxFullCount,
                int visionRejectCount,
                int visionJumpCount) {
            if (Double.isFinite(batteryVoltage)) {
                minBatteryVoltage = Math.min(minBatteryVoltage, batteryVoltage);
            }
            if (Double.isFinite(canBusUtilization)) {
                if (!Double.isFinite(maxCanBusUtilization) || canBusUtilization > maxCanBusUtilization) {
                    maxCanBusUtilization = canBusUtilization;
                }
            }
            if (Double.isFinite(canTxFullCount)) {
                if (!Double.isFinite(maxCanTxFullCount) || canTxFullCount > maxCanTxFullCount) {
                    maxCanTxFullCount = canTxFullCount;
                }
                canTxFullCountEnd = canTxFullCount;
            }
            visionRejectCountEnd = visionRejectCount;
            visionJumpCountEnd = visionJumpCount;

            lastBatteryLowState = batteryLow;
            if (brownout != lastBrownoutState) {
                healthEvents.add(String.format(Locale.US, "t=%.3f matchTime=%.2f brownout=%s", ts / 1_000_000.0, matchTime, brownout));
                lastBrownoutState = brownout;
            }
            if (gyroConnected != lastGyroConnectedState) {
                healthEvents.add(String.format(Locale.US, "t=%.3f matchTime=%.2f gyroConnected=%s", ts / 1_000_000.0, matchTime, gyroConnected));
                lastGyroConnectedState = gyroConnected;
            }
            if (visionConnected != lastVisionConnectedState) {
                healthEvents.add(String.format(Locale.US, "t=%.3f matchTime=%.2f visionConnected=%s", ts / 1_000_000.0, matchTime, visionConnected));
                lastVisionConnectedState = visionConnected;
            }
            if (cam0Connected != lastCam0ConnectedState) {
                healthEvents.add(String.format(Locale.US, "t=%.3f matchTime=%.2f cam0Connected=%s", ts / 1_000_000.0, matchTime, cam0Connected));
                lastCam0ConnectedState = cam0Connected;
            }
            if (cam1Connected != lastCam1ConnectedState) {
                healthEvents.add(String.format(Locale.US, "t=%.3f matchTime=%.2f cam1Connected=%s", ts / 1_000_000.0, matchTime, cam1Connected));
                lastCam1ConnectedState = cam1Connected;
            }
            if (odometryMismatch != lastOdometryMismatchState) {
                healthEvents.add(String.format(Locale.US, "t=%.3f matchTime=%.2f odometrySampleMismatch=%s", ts / 1_000_000.0, matchTime, odometryMismatch));
                lastOdometryMismatchState = odometryMismatch;
            }
        }

        private void observeCommandEvent(long ts, String lastCommandEvent) {
            if (lastCommandEvent == null || lastCommandEvent.isBlank()) {
                return;
            }
            commandEvents.add(String.format(Locale.US, "t=%.3f %s", ts / 1_000_000.0, lastCommandEvent));
            String commandName = extractCommandName(lastCommandEvent);
            if (!commandName.isBlank()) {
                startedCommandNames.add(commandName);
            }
            if (commandName.equals(initialSelectedCommand)) {
                if (lastCommandEvent.contains(" START ")) {
                    selectedAutoStarts++;
                    if (firstSelectedAutoStartUs == Long.MIN_VALUE) {
                        firstSelectedAutoStartUs = ts;
                    }
                } else if (lastCommandEvent.contains(" FINISH ")) {
                    selectedAutoFinishes++;
                } else if (lastCommandEvent.contains(" INTERRUPT ")) {
                    selectedAutoInterrupts++;
                }
            }
        }

        private void observePoseSample(
                long ts,
                double matchTime,
                Pose2d robotPose,
                Pose2d trajectorySetpoint,
                long trajectorySetpointTimestampUs,
                ChassisSpeeds requestedSpeeds,
                ChassisSpeeds measuredSpeeds,
                boolean automaticFeedEnabled,
                boolean shooterAtSetpoint,
                boolean aimReady,
                boolean gateOpen,
                String blockReason,
                String shootingState,
                double aimErrorDeg,
                boolean pathRotationOverrideEnabled,
                double kickerTorqueAmps,
                double transferCommandedPercent,
                boolean intakeExtended,
                boolean batteryLow) {
            poseSamples++;
            if (previousRobotPose != null) {
                drivenDistanceMeters += robotPose.getTranslation().getDistance(previousRobotPose.getTranslation());
            }
            previousRobotPose = robotPose;

            boolean pathSetpointFresh = trajectorySetpointTimestampUs != Long.MIN_VALUE
                    && ts - trajectorySetpointTimestampUs <= Math.round(TRAJECTORY_SETPOINT_FRESHNESS_SEC * 1_000_000.0);
            if (pathSetpointFresh) {
                double translationErrorMeters = robotPose.getTranslation().getDistance(trajectorySetpoint.getTranslation());
                double headingErrorDeg = Math.abs(Math.toDegrees(MathUtil.angleModulus(
                        robotPose.getRotation().minus(trajectorySetpoint.getRotation()).getRadians())));
                pathTranslationErrors.add(translationErrorMeters);
                pathHeadingErrorsDeg.add(headingErrorDeg);
                pathTrackedPoseSamples++;
                if (firstPathSampleUs == Long.MIN_VALUE) {
                    firstPathSampleUs = ts;
                    firstPathErrorMeters = translationErrorMeters;
                    firstPathHeadingErrorDeg = headingErrorDeg;
                }
                lastPathSampleUs = ts;
                lastPathErrorMeters = translationErrorMeters;
                lastPathHeadingErrorDeg = headingErrorDeg;
            }

            double requestedLinearSpeed = Math.hypot(requestedSpeeds.vxMetersPerSecond, requestedSpeeds.vyMetersPerSecond);
            double measuredLinearSpeed = Math.hypot(measuredSpeeds.vxMetersPerSecond, measuredSpeeds.vyMetersPerSecond);
            pathSpeedErrorsMps.add(Math.abs(requestedLinearSpeed - measuredLinearSpeed));
            if (requestedLinearSpeed >= HIGH_REQUEST_SPEED_THRESHOLD_MPS) {
                highRequestResponseRatios.add(measuredLinearSpeed / requestedLinearSpeed);
            }

            if (automaticFeedEnabled) {
                if (firstAutoFeedUs == Long.MIN_VALUE) {
                    firstAutoFeedUs = ts;
                }
                autoFeedPoseSamples++;
                if (Double.isFinite(aimErrorDeg)) {
                    aimErrorsWhileAutoFeedDeg.add(Math.abs(aimErrorDeg));
                }
                String normalizedBlockReason = normalize(blockReason);
                if (!normalizedBlockReason.isBlank()) {
                    blockReasonCounts.merge(normalizedBlockReason, 1, Integer::sum);
                }
            }
            if (shooterAtSetpoint) {
                shooterAtPoseSamples++;
            }
            if (aimReady) {
                aimReadyPoseSamples++;
            }
            if (gateOpen) {
                if (firstGateOpenUs == Long.MIN_VALUE) {
                    firstGateOpenUs = ts;
                }
                gateOpenPoseSamples++;
                if (Double.isFinite(aimErrorDeg)) {
                    aimErrorsWhileGateOpenDeg.add(Math.abs(aimErrorDeg));
                }
            }
            if (pathRotationOverrideEnabled) {
                pathRotationOverridePoseSamples++;
            }
            if (Math.abs(transferCommandedPercent) >= SIGNIFICANT_TRANSFER_PERCENT) {
                transferActivePoseSamples++;
            }
            if (Math.abs(kickerTorqueAmps) >= SIGNIFICANT_KICKER_TORQUE_AMPS) {
                kickerActivePoseSamples++;
            }
            if (intakeExtended) {
                intakeExtendedPoseSamples++;
            }
            if (lastBrownoutState) {
                brownoutPoseSamples++;
            }
            if (lastGyroConnectedState == false) {
                gyroDisconnectedPoseSamples++;
            }
            if (lastVisionConnectedState == false) {
                visionDisconnectedPoseSamples++;
            }
            if (lastCam0ConnectedState == false) {
                cam0DisconnectedPoseSamples++;
            }
            if (lastCam1ConnectedState == false) {
                cam1DisconnectedPoseSamples++;
            }
            if (lastOdometryMismatchState) {
                odometryMismatchPoseSamples++;
            }
            if (lastBatteryLowState) {
                batteryLowPoseSamples++;
            }
        }

        private void finish(
                long endUs,
                double endMatchTime,
                Pose2d finalRobotPose,
                ChassisSpeeds finalMeasuredSpeeds,
                String finalSelectedRoutine,
                String finalSelectedCommand) {
            this.endUs = endUs;
            this.finalRobotPose = finalRobotPose;
            this.finalMeasuredSpeeds = finalMeasuredSpeeds;
            this.finalSelectedRoutine = finalSelectedRoutine;
            this.finalSelectedCommand = finalSelectedCommand;
            evaluateIssues();
        }

        private void evaluateIssues() {
            if (autoExpectation.isPathPlannerAuto() && pathTrackedPoseSamples == 0) {
                issues.add("No fresh Odometry/TrajectorySetpoint samples were seen while autonomous was active.");
            }
            if (autoExpectation.resetOdom() && Double.isFinite(firstPathErrorMeters) && firstPathErrorMeters > 0.35) {
                issues.add(String.format(Locale.US, "Initial path translation error was high: %.3f m", firstPathErrorMeters));
            }
            if (autoExpectation.resetOdom() && Double.isFinite(firstPathHeadingErrorDeg) && firstPathHeadingErrorDeg > 15.0) {
                issues.add(String.format(Locale.US, "Initial path heading error was high: %.1f deg", firstPathHeadingErrorDeg));
            }
            if (pathTranslationErrors.count() > 0 && pathTranslationErrors.percentile(95.0) > 0.45) {
                issues.add(String.format(Locale.US, "Path translation error p95 was high: %.3f m", pathTranslationErrors.percentile(95.0)));
            }
            if (pathTranslationErrors.count() > 0 && pathTranslationErrors.max() > 0.90) {
                issues.add(String.format(Locale.US, "Path translation error max was high: %.3f m", pathTranslationErrors.max()));
            }
            if (Double.isFinite(lastPathErrorMeters) && lastPathErrorMeters > 0.45) {
                issues.add(String.format(Locale.US, "Final path translation error remained high: %.3f m", lastPathErrorMeters));
            }
            if (highRequestResponseRatios.count() > 0 && highRequestResponseRatios.percentile(10.0) < 0.25) {
                issues.add(String.format(Locale.US, "Drive response ratio p10 under high request was low: %.2f", highRequestResponseRatios.percentile(10.0)));
            }
            if (brownoutPoseSamples > 0) {
                issues.add("Brownout was observed during autonomous.");
            }
            if (gyroDisconnectedPoseSamples > 0) {
                issues.add("Gyro disconnected during autonomous.");
            }
            if (visionDisconnectedPoseSamples > 0) {
                issues.add("All vision cameras were disconnected during autonomous.");
            }
            if (cam0DisconnectedPoseSamples > 0 || cam1DisconnectedPoseSamples > 0) {
                issues.add("At least one vision camera disconnected during autonomous.");
            }
            if (odometryMismatchPoseSamples > 0) {
                issues.add("Drive odometry sample-count mismatch occurred during autonomous.");
            }
            if (Double.isFinite(maxCanBusUtilization) && maxCanBusUtilization > 0.95) {
                issues.add(String.format(Locale.US, "CANivore bus utilization exceeded 95%%: %.3f", maxCanBusUtilization));
            }
            if (Double.isFinite(canTxFullCountStart)
                    && Double.isFinite(canTxFullCountEnd)
                    && canTxFullCountEnd - canTxFullCountStart > 0.5) {
                issues.add(String.format(Locale.US, "CANivore TxFullCount increased during autonomous by %.0f", canTxFullCountEnd - canTxFullCountStart));
            }
            boolean shootExpected = autoExpectation.allNamedCommands().stream().anyMatch(name -> name.startsWith("ShooterShootHub"));
            boolean movingShotExpected = autoExpectation.allNamedCommands().stream().anyMatch(name -> name.contains("OnMove"));
            if (shootExpected && autoFeedPoseSamples == 0) {
                issues.add("Auto declares a shooter command, but AutomaticFeedEnabled never became true.");
            }
            if (shootExpected && gateOpenPoseSamples == 0) {
                issues.add("Auto declares a shooter command, but the feed gate never opened.");
            }
            if (movingShotExpected && pathRotationOverridePoseSamples == 0) {
                issues.add("Moving-shot auto never enabled the PathPlanner rotation-feedback override.");
            }
        }

        private String format() {
            StringBuilder out = new StringBuilder();
            out.append(String.format(Locale.US, "  start_t=%.3f s%n", startUs / 1_000_000.0));
            if (endUs != Long.MIN_VALUE) {
                out.append(String.format(Locale.US, "  end_t=%.3f s%n", endUs / 1_000_000.0));
                out.append(String.format(Locale.US, "  duration=%.3f s%n", (endUs - startUs) / 1_000_000.0));
            } else {
                out.append("  end_t=<log end>\n");
                out.append("  duration=<open-ended>\n");
            }
            out.append(String.format(Locale.US, "  start_match_time=%.2f%n", startMatchTime));
            out.append("  selected_routine.initial=").append(normalize(initialSelectedRoutine)).append('\n');
            out.append("  selected_command.initial=").append(normalize(initialSelectedCommand)).append('\n');
            out.append("  selected_routine.final=").append(normalize(finalSelectedRoutine)).append('\n');
            out.append("  selected_command.final=").append(normalize(finalSelectedCommand)).append('\n');
            out.append("  auto_expectation.name=").append(normalize(autoExpectation.autoName())).append('\n');
            out.append("  auto_expectation.resetOdom=").append(autoExpectation.resetOdom()).append('\n');
            out.append("  auto_expectation.paths=").append(autoExpectation.pathNames()).append('\n');
            out.append("  auto_expectation.topLevelNamedCommands=").append(autoExpectation.topLevelNamedCommands()).append('\n');
            out.append("  auto_expectation.pathEventNamedCommands=").append(autoExpectation.pathEventNamedCommands()).append('\n');
            out.append("  auto_expectation.allNamedCommands=").append(autoExpectation.allNamedCommands()).append('\n');
            out.append("  auto_expectation.file=")
                    .append(autoExpectation.autoFile() == null ? "<none>" : autoExpectation.autoFile().toAbsolutePath())
                    .append('\n');
            out.append(String.format(Locale.US, "  selected_auto_starts=%d%n", selectedAutoStarts));
            out.append(String.format(Locale.US, "  selected_auto_finishes=%d%n", selectedAutoFinishes));
            out.append(String.format(Locale.US, "  selected_auto_interrupts=%d%n", selectedAutoInterrupts));
            if (firstSelectedAutoStartUs != Long.MIN_VALUE) {
                out.append(String.format(Locale.US, "  selected_auto_start_delay=%.3f s%n", (firstSelectedAutoStartUs - startUs) / 1_000_000.0));
            } else {
                out.append("  selected_auto_start_delay=<not observed>\n");
            }
            out.append(String.format(Locale.US, "  driven_distance=%.3f m%n", drivenDistanceMeters));
            out.append(String.format(Locale.US, "  final_pose=(%.3f, %.3f, %.1f deg)%n", finalRobotPose.getX(), finalRobotPose.getY(), finalRobotPose.getRotation().getDegrees()));
            out.append(String.format(Locale.US, "  final_measured_speeds=(vx=%.3f, vy=%.3f, omega=%.3f)%n", finalMeasuredSpeeds.vxMetersPerSecond, finalMeasuredSpeeds.vyMetersPerSecond, finalMeasuredSpeeds.omegaRadiansPerSecond));
            out.append(String.format(Locale.US, "  pose_samples=%d%n", poseSamples));
            out.append(String.format(Locale.US, "  path_tracked_pose_samples=%d%n", pathTrackedPoseSamples));
            out.append(String.format(Locale.US, "  first_path_error=%.3f m heading=%.1f deg%n", firstPathErrorMeters, firstPathHeadingErrorDeg));
            out.append(String.format(Locale.US, "  last_path_error=%.3f m heading=%.1f deg%n", lastPathErrorMeters, lastPathHeadingErrorDeg));
            out.append(String.format(Locale.US, "  first_auto_feed_t=%s%n", AutonomousWindow.formatTimestamp(firstAutoFeedUs)));
            out.append(String.format(Locale.US, "  first_gate_open_t=%s%n", AutonomousWindow.formatTimestamp(firstGateOpenUs)));
            out.append(String.format(Locale.US, "  last_path_sample_t=%s%n", AutonomousWindow.formatTimestamp(lastPathSampleUs)));
            out.append(String.format(Locale.US, "  path_end_to_first_auto_feed=%s%n", AutonomousWindow.formatDelta(lastPathSampleUs, firstAutoFeedUs)));
            out.append(String.format(Locale.US, "  path_end_to_first_gate_open=%s%n", AutonomousWindow.formatDelta(lastPathSampleUs, firstGateOpenUs)));
            out.append("  path_translation_error=").append(pathTranslationErrors.summaryMeters()).append('\n');
            out.append("  path_heading_error=").append(pathHeadingErrorsDeg.summaryDegrees()).append('\n');
            out.append("  requested_vs_measured_speed_error=").append(pathSpeedErrorsMps.summaryMetersPerSecond()).append('\n');
            out.append("  high_request_response_ratio=").append(highRequestResponseRatios.summaryRatio()).append('\n');
            out.append(String.format(Locale.US, "  min_battery_voltage=%.3f V%n", minBatteryVoltage));
            out.append(String.format(Locale.US, "  max_canivore_bus_util=%.3f%n", maxCanBusUtilization));
            out.append(String.format(Locale.US, "  canivore_tx_full_delta=%.0f%n", canTxFullCountEnd - canTxFullCountStart));
            out.append(String.format(Locale.US, "  vision_reject_delta=%d%n", visionRejectCountEnd - visionRejectCountStart));
            out.append(String.format(Locale.US, "  vision_jump_delta=%d%n", visionJumpCountEnd - visionJumpCountStart));
            out.append(String.format(Locale.US, "  autoFeed_pose_samples=%d%n", autoFeedPoseSamples));
            out.append(String.format(Locale.US, "  shooterAtSetpoint_pose_samples=%d%n", shooterAtPoseSamples));
            out.append(String.format(Locale.US, "  aimReady_pose_samples=%d%n", aimReadyPoseSamples));
            out.append(String.format(Locale.US, "  gateOpen_pose_samples=%d%n", gateOpenPoseSamples));
            out.append(String.format(Locale.US, "  pathRotationOverride_pose_samples=%d%n", pathRotationOverridePoseSamples));
            out.append(String.format(Locale.US, "  transferActive_pose_samples=%d%n", transferActivePoseSamples));
            out.append(String.format(Locale.US, "  kickerActive_pose_samples=%d%n", kickerActivePoseSamples));
            out.append(String.format(Locale.US, "  intakeExtended_pose_samples=%d%n", intakeExtendedPoseSamples));
            out.append(String.format(Locale.US, "  brownout_pose_samples=%d%n", brownoutPoseSamples));
            out.append(String.format(Locale.US, "  batteryLow_pose_samples=%d%n", batteryLowPoseSamples));
            out.append(String.format(Locale.US, "  gyroDisconnected_pose_samples=%d%n", gyroDisconnectedPoseSamples));
            out.append(String.format(Locale.US, "  visionDisconnected_pose_samples=%d%n", visionDisconnectedPoseSamples));
            out.append(String.format(Locale.US, "  cam0Disconnected_pose_samples=%d%n", cam0DisconnectedPoseSamples));
            out.append(String.format(Locale.US, "  cam1Disconnected_pose_samples=%d%n", cam1DisconnectedPoseSamples));
            out.append(String.format(Locale.US, "  odometryMismatch_pose_samples=%d%n", odometryMismatchPoseSamples));
            out.append("  aim_error_while_auto_feed=").append(aimErrorsWhileAutoFeedDeg.summaryDegrees()).append('\n');
            out.append("  aim_error_while_gate_open=").append(aimErrorsWhileGateOpenDeg.summaryDegrees()).append('\n');
            out.append("  block_reasons_during_auto_feed=").append(sortMap(blockReasonCounts)).append('\n');
            out.append("  started_commands=").append(startedCommandNames).append('\n');
            out.append("  issues\n");
            appendLines(out, issues);
            out.append("  health_events\n");
            appendLines(out, healthEvents);
            out.append("  command_events\n");
            appendLines(out, commandEvents);
            return out.toString();
        }

        private static String extractCommandName(String event) {
            int nameIdx = event.indexOf(" name=");
            if (nameIdx < 0) {
                return "";
            }
            int from = nameIdx + " name=".length();
            int to = event.indexOf(" source=", from);
            if (to < 0) {
                to = event.indexOf(" requirements=", from);
            }
            if (to < 0) {
                to = event.length();
            }
            return event.substring(from, to).trim();
        }

        private static void appendLines(StringBuilder out, List<String> lines) {
            if (lines.isEmpty()) {
                out.append("    <none>\n");
                return;
            }
            for (String line : lines) {
                out.append("    ").append(line).append('\n');
            }
        }

        private static String formatTimestamp(long timestampUs) {
            return timestampUs == Long.MIN_VALUE
                    ? "<none>"
                    : String.format(Locale.US, "%.3f s", timestampUs / 1_000_000.0);
        }

        private static String formatDelta(long fromUs, long toUs) {
            if (fromUs == Long.MIN_VALUE || toUs == Long.MIN_VALUE) {
                return "<none>";
            }
            return String.format(Locale.US, "%.3f s", (toUs - fromUs) / 1_000_000.0);
        }
    }

    private static final class Distribution {
        private final List<Double> values = new ArrayList<>();

        private void add(double value) {
            if (Double.isFinite(value)) {
                values.add(value);
            }
        }

        private int count() {
            return values.size();
        }

        private double max() {
            if (values.isEmpty()) {
                return Double.NaN;
            }
            double max = Double.NEGATIVE_INFINITY;
            for (double value : values) {
                max = Math.max(max, value);
            }
            return max;
        }

        private double mean() {
            if (values.isEmpty()) {
                return Double.NaN;
            }
            double sum = 0.0;
            for (double value : values) {
                sum += value;
            }
            return sum / values.size();
        }

        private double percentile(double percentile) {
            if (values.isEmpty()) {
                return Double.NaN;
            }
            List<Double> sorted = new ArrayList<>(values);
            Collections.sort(sorted);
            double rank = percentile / 100.0 * (sorted.size() - 1);
            int lower = (int) Math.floor(rank);
            int upper = (int) Math.ceil(rank);
            if (lower == upper) {
                return sorted.get(lower);
            }
            double fraction = rank - lower;
            return sorted.get(lower) * (1.0 - fraction) + sorted.get(upper) * fraction;
        }

        private String summaryMeters() {
            return summary("m");
        }

        private String summaryMetersPerSecond() {
            return summary("mps");
        }

        private String summaryDegrees() {
            return summary("deg");
        }

        private String summaryRatio() {
            return summary("");
        }

        private String summary(String unit) {
            if (values.isEmpty()) {
                return "<none>";
            }
            String suffix = unit == null || unit.isBlank() ? "" : " " + unit;
            return String.format(
                    Locale.US,
                    "count=%d mean=%.3f%s p50=%.3f%s p90=%.3f%s p95=%.3f%s max=%.3f%s",
                    count(),
                    mean(),
                    suffix,
                    percentile(50.0),
                    suffix,
                    percentile(90.0),
                    suffix,
                    percentile(95.0),
                    suffix,
                    max(),
                    suffix);
        }
    }

    private static String normalize(String value) {
        return value == null || value.isBlank() ? "<none>" : value;
    }

    private static Map<String, Integer> sortMap(Map<String, Integer> map) {
        return map.entrySet().stream()
                .sorted(Map.Entry.<String, Integer>comparingByValue().reversed().thenComparing(Map.Entry.comparingByKey()))
                .collect(java.util.stream.Collectors.toMap(
                        Map.Entry::getKey,
                        Map.Entry::getValue,
                        (left, right) -> left,
                        java.util.LinkedHashMap::new));
    }
}
