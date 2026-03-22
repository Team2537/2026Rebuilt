package frc.robot.diagnostics;

import static org.junit.jupiter.api.Assertions.assertFalse;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.util.struct.StructBuffer;
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
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import org.junit.jupiter.api.Test;

class VisionOdometryLogAnalyzerTest {
    private static final Pattern QUAL_MATCH_PATTERN = Pattern.compile("_q(\\d+)\\.wpilog$");

    @Test
    void analyzeVisionAndOdometryForMdbetMatchesAfterQ7() throws Exception {
        Path logsDir = Path.of("logs/mdbet").toAbsolutePath();
        assertFalse(!Files.isDirectory(logsDir), "Missing log directory: " + logsDir);

        List<Path> logs = Files.list(logsDir)
                .filter(path -> path.getFileName().toString().endsWith(".wpilog"))
                .filter(path -> qualifyAfterMatch7(path.getFileName().toString()))
                .sorted()
                .toList();
        assertFalse(logs.isEmpty(), "No MDBET qualification logs found after q7 in " + logsDir);

        List<LogAnalysis> analyses = new ArrayList<>();
        for (Path log : logs) {
            analyses.add(analyze(log));
        }

        String report = formatCombinedReport(analyses);
        Path out = Path.of("build/reports/diagnostics/vision-odometry-mdbet-report.txt").toAbsolutePath();
        Files.createDirectories(out.getParent());
        Files.writeString(out, report);

        System.out.println(report);
        System.out.println("Vision/odometry report written: " + out);
    }

    private static boolean qualifyAfterMatch7(String fileName) {
        Matcher matcher = QUAL_MATCH_PATTERN.matcher(fileName);
        return matcher.find() && Integer.parseInt(matcher.group(1)) > 7;
    }

    private static LogAnalysis analyze(Path wpilog) throws IOException {
        DataLogReader reader = new DataLogReader(wpilog.toString());
        if (!reader.isValid()) {
            throw new IllegalStateException("Invalid WPILOG file: " + wpilog);
        }

        Map<Integer, EntryInfo> entries = readEntries(reader);
        EntryLookup entry = new EntryLookup(entries);
        StructBuffer<Pose2d> poseBuffer = StructBuffer.create(Pose2d.struct);
        StructBuffer<ChassisSpeeds> chassisBuffer = StructBuffer.create(ChassisSpeeds.struct);

        LogAnalysis stats = new LogAnalysis(wpilog, entries);

        int enabledEntry = entry.find("RobotState/Enabled");
        int modeEntry = entry.find("RobotState/Mode");
        int matchTimeEntry = entry.find("RobotState/MatchTime");
        int visionDisabledEntry = entry.find("RobotState/VisionDisabledOverride");
        int visionConnectedEntry = entry.find("RobotState/VisionConnected");
        int camera0ConnectedEntry = entry.find("RobotState/Vision/Camera0Connected");
        int camera1ConnectedEntry = entry.find("RobotState/Vision/Camera1Connected");
        int gyroConnectedEntry = entry.find("RobotState/GyroConnected");

        int odomPoseEntry = entry.find("Odometry/Robot");
        int measuredSpeedsEntry = entry.find("SwerveChassisSpeeds/Measured");
        int sharedSampleCountEntry = entry.find("Drive/Odometry/SharedSampleCount");
        int sampleCountMismatchEntry = entry.find("Drive/Odometry/SampleCountMismatch");
        int droppedPhoenixEntry = entry.find("Drive/OdometryThread/DroppedPhoenixSamples");
        int droppedTimestampEntry = entry.find("Drive/OdometryThread/DroppedTimestampSamples");

        int unifiedPoseEntry = entry.find("Vision/unifiedRobotPose");
        int unifiedValidEntry = entry.find("Vision/unifiedRobotPoseValid");
        int rawPoseEntry = entry.find("Vision/unifiedRobotPoseRaw");
        int rawValidEntry = entry.find("Vision/unifiedRobotPoseRawValid");
        int visibleTagsEntry = entry.find("Vision/visibleTagIds");

        int eventSequenceEntry = entry.find("Vision/events/sequence");
        int rejectCountEntry = entry.find("Vision/events/rejectCount");
        int jumpCountEntry = entry.find("Vision/events/jumpCount");
        int eventTypeEntry = entry.find("Vision/events/last/type");
        int eventTimestampSecondsEntry = entry.find("Vision/events/last/timestampSeconds");
        int eventCameraEntry = entry.find("Vision/events/last/cameraIndex");
        int eventMeasuredPoseEntry = entry.find("Vision/events/last/measuredPose");
        int eventOdometryPoseEntry = entry.find("Vision/events/last/odometryPose");
        int eventTranslationDeltaEntry = entry.find("Vision/events/last/translationDeltaMeters");
        int eventHeadingDeltaEntry = entry.find("Vision/events/last/headingDeltaDegrees");
        int eventTagCountEntry = entry.find("Vision/events/last/tagCount");
        int eventTagIdsEntry = entry.find("Vision/events/last/tagIds");
        int eventAmbiguityEntry = entry.find("Vision/events/last/ambiguity");
        int eventAvgDistanceEntry = entry.find("Vision/events/last/avgDistanceMeters");
        int eventLinearStdDevEntry = entry.find("Vision/events/last/linearStdDev");
        int eventAngularStdDevEntry = entry.find("Vision/events/last/angularStdDev");

        State state = new State();
        PendingVisionEvent pendingVisionEvent = null;

        Long currentTimestamp = null;
        long lastTimestamp = 0L;
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

            long recordTimestamp = record.getTimestamp();
            if (currentTimestamp == null) {
                currentTimestamp = recordTimestamp;
                stats.firstTimestampMicros = recordTimestamp;
            } else {
                long gapMicros = recordTimestamp - currentTimestamp.longValue();
                if (gapMicros < 0L) {
                    if (gapMicros < -100_000L) {
                        stats.anomalies.add(String.format(
                                Locale.US,
                                "Large negative timestamp reordering current=%.3f next=%.3f gap=%.3fs",
                                currentTimestamp / 1_000_000.0,
                                recordTimestamp / 1_000_000.0,
                                gapMicros / 1_000_000.0));
                    }
                    recordTimestamp = currentTimestamp;
                    gapMicros = 0L;
                }
                if (gapMicros > 600_000_000L) {
                    stats.anomalies.add(String.format(
                            Locale.US,
                            "Stopped at suspicious timestamp jump current=%.3f next=%.3f gap=%.3fs",
                            currentTimestamp / 1_000_000.0,
                            recordTimestamp / 1_000_000.0,
                            gapMicros / 1_000_000.0));
                    break;
                }
                if (recordTimestamp != currentTimestamp.longValue()) {
                    if (pendingVisionEvent != null) {
                        stats.visionEvents.add(pendingVisionEvent.finish(state));
                        pendingVisionEvent = null;
                    }
                    stats.advanceWindow(currentTimestamp, recordTimestamp, state);
                    currentTimestamp = recordTimestamp;
                }
            }
            lastTimestamp = recordTimestamp;

            if (record.isStart() || record.isControl()) {
                continue;
            }

            int e = record.getEntry();
            double tsSec = recordTimestamp / 1_000_000.0;

            if (e == enabledEntry) {
                boolean newValue = readBooleanLenient(record);
                if (newValue != state.enabled) {
                    state.enabled = newValue;
                    stats.transitionEvents.add(snapshot("enabled=" + newValue, tsSec, state));
                }
            } else if (e == modeEntry) {
                String newValue = record.getString();
                if (!newValue.equals(state.mode)) {
                    state.mode = newValue;
                    stats.transitionEvents.add(snapshot("mode=" + newValue, tsSec, state));
                }
            } else if (e == matchTimeEntry) {
                state.matchTime = record.getDouble();
            } else if (e == visionDisabledEntry) {
                boolean newValue = readBooleanLenient(record);
                if (newValue != state.visionDisabledOverride) {
                    state.visionDisabledOverride = newValue;
                    stats.transitionEvents.add(snapshot("visionDisabledOverride=" + newValue, tsSec, state));
                }
            } else if (e == visionConnectedEntry) {
                boolean newValue = readBooleanLenient(record);
                if (newValue != state.visionConnected) {
                    state.visionConnected = newValue;
                    stats.visionConnectedTransitions++;
                    stats.connectionEvents.add(snapshot("visionConnected=" + newValue, tsSec, state));
                }
            } else if (e == camera0ConnectedEntry) {
                boolean newValue = readBooleanLenient(record);
                if (newValue != state.cameraConnected[0]) {
                    state.cameraConnected[0] = newValue;
                    stats.cameraConnectedTransitions[0]++;
                    stats.connectionEvents.add(snapshot("camera0Connected=" + newValue, tsSec, state));
                }
            } else if (e == camera1ConnectedEntry) {
                boolean newValue = readBooleanLenient(record);
                if (newValue != state.cameraConnected[1]) {
                    state.cameraConnected[1] = newValue;
                    stats.cameraConnectedTransitions[1]++;
                    stats.connectionEvents.add(snapshot("camera1Connected=" + newValue, tsSec, state));
                }
            } else if (e == gyroConnectedEntry) {
                boolean newValue = readBooleanLenient(record);
                if (newValue != state.gyroConnected) {
                    state.gyroConnected = newValue;
                    stats.gyroConnectedTransitions++;
                    stats.connectionEvents.add(snapshot("gyroConnected=" + newValue, tsSec, state));
                }
            } else if (e == odomPoseEntry) {
                state.odometryPose = poseBuffer.read(record.getRaw());
                stats.observeOdometryPose(tsSec, state);
            } else if (e == measuredSpeedsEntry) {
                state.measuredSpeeds = chassisBuffer.read(record.getRaw());
            } else if (e == sharedSampleCountEntry) {
                state.sharedSampleCount = (int) readLongLenient(record);
                stats.sharedSampleCounts.add(state.sharedSampleCount);
            } else if (e == sampleCountMismatchEntry) {
                state.sampleCountMismatch = readBooleanLenient(record);
            } else if (e == droppedPhoenixEntry) {
                state.droppedPhoenixSamples = readLongLenient(record);
                stats.maxDroppedPhoenixSamples = Math.max(stats.maxDroppedPhoenixSamples, state.droppedPhoenixSamples);
            } else if (e == droppedTimestampEntry) {
                state.droppedTimestampSamples = readLongLenient(record);
                stats.maxDroppedTimestampSamples = Math.max(stats.maxDroppedTimestampSamples, state.droppedTimestampSamples);
            } else if (e == unifiedPoseEntry) {
                state.unifiedPose = poseBuffer.read(record.getRaw());
                state.unifiedPoseLastLogTimeSec = tsSec;
                if (state.unifiedPoseValid) {
                    stats.observeUnifiedPose(tsSec, state, false);
                }
            } else if (e == unifiedValidEntry) {
                boolean newValue = readBooleanLenient(record);
                if (newValue != state.unifiedPoseValid) {
                    state.unifiedPoseValid = newValue;
                    stats.transitionEvents.add(snapshot("unifiedPoseValid=" + newValue, tsSec, state));
                }
            } else if (e == rawPoseEntry) {
                state.rawPose = poseBuffer.read(record.getRaw());
                state.rawPoseLastLogTimeSec = tsSec;
                if (state.rawPoseValid) {
                    stats.observeUnifiedPose(tsSec, state, true);
                }
            } else if (e == rawValidEntry) {
                boolean newValue = readBooleanLenient(record);
                if (newValue != state.rawPoseValid) {
                    state.rawPoseValid = newValue;
                    stats.transitionEvents.add(snapshot("rawPoseValid=" + newValue, tsSec, state));
                }
            } else if (e == visibleTagsEntry) {
                state.visibleTagIds = readLongArrayLenient(record);
                stats.maxVisibleTagCount = Math.max(stats.maxVisibleTagCount, state.visibleTagIds.length);
            } else if (e == eventSequenceEntry) {
                long newSequence = readLongLenient(record);
                if (newSequence > state.visionEventSequence) {
                    if (pendingVisionEvent != null) {
                        stats.visionEvents.add(pendingVisionEvent.finish(state));
                    }
                    pendingVisionEvent = new PendingVisionEvent(newSequence, tsSec, state);
                }
                state.visionEventSequence = newSequence;
            } else if (e == rejectCountEntry) {
                state.rejectCount = readLongLenient(record);
            } else if (e == jumpCountEntry) {
                state.jumpCount = readLongLenient(record);
            } else if (e == eventTypeEntry) {
                String value = record.getString();
                state.lastVisionEventType = value;
                if (pendingVisionEvent != null) {
                    pendingVisionEvent.type = value;
                }
            } else if (e == eventTimestampSecondsEntry) {
                double value = record.getDouble();
                state.lastVisionEventTimestampSeconds = value;
                if (pendingVisionEvent != null) {
                    pendingVisionEvent.observationTimestampSeconds = value;
                }
            } else if (e == eventCameraEntry) {
                int value = (int) readLongLenient(record);
                state.lastVisionEventCamera = value;
                if (pendingVisionEvent != null) {
                    pendingVisionEvent.cameraIndex = value;
                }
            } else if (e == eventMeasuredPoseEntry) {
                Pose2d value = poseBuffer.read(record.getRaw());
                state.lastVisionEventMeasuredPose = value;
                if (pendingVisionEvent != null) {
                    pendingVisionEvent.measuredPose = value;
                    pendingVisionEvent.measuredPoseSeen = true;
                }
            } else if (e == eventOdometryPoseEntry) {
                Pose2d value = poseBuffer.read(record.getRaw());
                state.lastVisionEventOdometryPose = value;
                if (pendingVisionEvent != null) {
                    pendingVisionEvent.odometryPose = value;
                    pendingVisionEvent.odometryPoseSeen = true;
                }
            } else if (e == eventTranslationDeltaEntry) {
                double value = record.getDouble();
                state.lastVisionEventTranslationDeltaMeters = value;
                if (pendingVisionEvent != null) {
                    pendingVisionEvent.translationDeltaMeters = value;
                }
            } else if (e == eventHeadingDeltaEntry) {
                double value = record.getDouble();
                state.lastVisionEventHeadingDeltaDegrees = value;
                if (pendingVisionEvent != null) {
                    pendingVisionEvent.headingDeltaDegrees = value;
                }
            } else if (e == eventTagCountEntry) {
                int value = (int) readLongLenient(record);
                state.lastVisionEventTagCount = value;
                if (pendingVisionEvent != null) {
                    pendingVisionEvent.tagCount = value;
                }
            } else if (e == eventTagIdsEntry) {
                long[] value = readLongArrayLenient(record);
                state.lastVisionEventTagIds = value;
                if (pendingVisionEvent != null) {
                    pendingVisionEvent.tagIds = value;
                }
            } else if (e == eventAmbiguityEntry) {
                double value = record.getDouble();
                state.lastVisionEventAmbiguity = value;
                if (pendingVisionEvent != null) {
                    pendingVisionEvent.ambiguity = value;
                }
            } else if (e == eventAvgDistanceEntry) {
                double value = record.getDouble();
                state.lastVisionEventAvgDistanceMeters = value;
                if (pendingVisionEvent != null) {
                    pendingVisionEvent.avgDistanceMeters = value;
                }
            } else if (e == eventLinearStdDevEntry) {
                double value = record.getDouble();
                state.lastVisionEventLinearStdDev = value;
                if (pendingVisionEvent != null) {
                    pendingVisionEvent.linearStdDev = value;
                }
            } else if (e == eventAngularStdDevEntry) {
                double value = record.getDouble();
                state.lastVisionEventAngularStdDev = value;
                if (pendingVisionEvent != null) {
                    pendingVisionEvent.angularStdDev = value;
                }
            }
        }

        if (pendingVisionEvent != null) {
            stats.visionEvents.add(pendingVisionEvent.finish(state));
        }
        if (currentTimestamp != null) {
            stats.advanceWindow(currentTimestamp, lastTimestamp, state);
            stats.lastTimestampMicros = lastTimestamp;
        }
        stats.finalizeSummary(state);
        return stats;
    }

    private static Map<Integer, EntryInfo> readEntries(DataLogReader reader) {
        Map<Integer, EntryInfo> entries = new HashMap<>();
        Iterator<DataLogRecord> iterator = reader.iterator();
        while (true) {
            try {
                if (!iterator.hasNext()) {
                    break;
                }
                DataLogRecord record = iterator.next();
                if (!record.isStart()) {
                    continue;
                }
                var start = record.getStartData();
                entries.put(start.entry, new EntryInfo(start.name, start.type));
            } catch (IllegalArgumentException ignored) {
                break;
            }
        }
        return entries;
    }

    private static String formatCombinedReport(List<LogAnalysis> analyses) {
        AggregateAnalysis aggregate = new AggregateAnalysis(analyses);
        StringBuilder out = new StringBuilder();
        out.append("MDBET vision + odometry analysis (qualification logs after q7)\n");
        out.append("logs analyzed: ").append(analyses.size()).append("\n\n");

        out.append("one-line summary\n");
        for (LogAnalysis analysis : analyses) {
            out.append("  ")
                    .append(analysis.path.getFileName())
                    .append(" enabled=")
                    .append(format1(analysis.enabledTracker.totalSec))
                    .append("s visionDisc=")
                    .append(format1(analysis.visionDisconnectedTracker.totalSec))
                    .append("s cam0Disc=")
                    .append(format1(analysis.cameraDisconnectedTrackers[0].totalSec))
                    .append("s cam1Disc=")
                    .append(format1(analysis.cameraDisconnectedTrackers[1].totalSec))
                    .append("s unifiedValid=")
                    .append(percent(analysis.unifiedValidTracker.totalSec, analysis.enabledTracker.totalSec))
                    .append(" rawValid=")
                    .append(percent(analysis.rawValidTracker.totalSec, analysis.enabledTracker.totalSec))
                    .append(" zeroShared=")
                    .append(format1(analysis.sharedSampleZeroTracker.totalSec))
                    .append("s mismatch=")
                    .append(format1(analysis.sampleMismatchTracker.totalSec))
                    .append("s rejects=")
                    .append(analysis.finalRejectCount)
                    .append(" jumps=")
                    .append(analysis.finalJumpCount)
                    .append(" maxAcceptedΔ=")
                    .append(format2(analysis.acceptedTranslationDelta.max()))
                    .append("m")
                    .append(" maxRawΔ=")
                    .append(format2(analysis.rawTranslationDelta.max()))
                    .append("m")
                    .append(" maxOdomStepSpeed=")
                    .append(format2(analysis.odometryImpliedLinearSpeed.max()))
                    .append("m/s\n");
        }
        out.append('\n');

        out.append("aggregate summary\n");
        out.append(aggregate.format()).append('\n');

        for (LogAnalysis analysis : analyses) {
            out.append(analysis.format()).append('\n');
        }
        return out.toString();
    }

    private static String snapshot(String label, double tsSec, State state) {
        return String.format(
                Locale.US,
                "%s t=%.3f mode=%s matchTime=%.1f enabled=%s gyro=%s vision=%s cam0=%s cam1=%s visionDisabled=%s visibleTags=%s odom=%s unifiedValid=%s unified=%s rawValid=%s raw=%s meas=(%.2f,%.2f,%.1fdeg/s) shared=%d mismatch=%s",
                label,
                tsSec,
                state.mode,
                state.matchTime,
                state.enabled,
                state.gyroConnected,
                state.visionConnected,
                state.cameraConnected[0],
                state.cameraConnected[1],
                state.visionDisabledOverride,
                formatTagIds(state.visibleTagIds),
                formatPose(state.odometryPose),
                state.unifiedPoseValid,
                formatPose(state.unifiedPose),
                state.rawPoseValid,
                formatPose(state.rawPose),
                state.measuredSpeeds.vxMetersPerSecond,
                state.measuredSpeeds.vyMetersPerSecond,
                Math.toDegrees(state.measuredSpeeds.omegaRadiansPerSecond),
                state.sharedSampleCount,
                state.sampleCountMismatch);
    }

    private static String formatPose(Pose2d pose) {
        if (pose == null) {
            return "<null>";
        }
        return String.format(Locale.US, "(%.2f,%.2f,%.1fdeg)", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
    }

    private static String formatTagIds(long[] tagIds) {
        if (tagIds == null || tagIds.length == 0) {
            return "[]";
        }
        return Arrays.toString(tagIds);
    }

    private static boolean readBooleanLenient(DataLogRecord record) {
        try {
            return record.getBoolean();
        } catch (Exception ignored) {
        }
        try {
            return Boolean.parseBoolean(record.getString());
        } catch (Exception ignored) {
        }
        return readLongLenient(record) != 0L;
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

    private static long[] readLongArrayLenient(DataLogRecord record) {
        try {
            return record.getIntegerArray();
        } catch (Exception ignored) {
        }
        return new long[0];
    }

    private static double translationDelta(Pose2d a, Pose2d b) {
        if (a == null || b == null) {
            return Double.NaN;
        }
        return a.getTranslation().getDistance(b.getTranslation());
    }

    private static double headingDeltaDegrees(Rotation2d a, Rotation2d b) {
        if (a == null || b == null) {
            return Double.NaN;
        }
        return Math.abs(Math.IEEEremainder(a.minus(b).getDegrees(), 360.0));
    }

    private static String format1(double value) {
        return String.format(Locale.US, "%.1f", value);
    }

    private static String format2(double value) {
        if (!Double.isFinite(value)) {
            return "n/a";
        }
        return String.format(Locale.US, "%.2f", value);
    }

    private static String percent(double numerator, double denominator) {
        if (!(denominator > 0.0) || !Double.isFinite(numerator)) {
            return "n/a";
        }
        return String.format(Locale.US, "%.1f%%", numerator * 100.0 / denominator);
    }

    private record EntryInfo(String name, String type) {}

    private static final class EntryLookup {
        private final Map<Integer, EntryInfo> entries;

        private EntryLookup(Map<Integer, EntryInfo> entries) {
            this.entries = entries;
        }

        private int find(String... suffixes) {
            for (String suffix : suffixes) {
                for (var entry : entries.entrySet()) {
                    if (entry.getValue().name.endsWith(suffix)) {
                        return entry.getKey();
                    }
                }
            }
            return -1;
        }
    }

    private static final class State {
        private boolean enabled;
        private String mode = "";
        private double matchTime = Double.NaN;
        private boolean visionDisabledOverride;
        private boolean visionConnected = true;
        private final boolean[] cameraConnected = new boolean[] { true, true };
        private boolean gyroConnected = true;

        private Pose2d odometryPose = new Pose2d();
        private Pose2d unifiedPose = new Pose2d();
        private boolean unifiedPoseValid;
        private double unifiedPoseLastLogTimeSec = Double.NaN;
        private Pose2d rawPose = new Pose2d();
        private boolean rawPoseValid;
        private double rawPoseLastLogTimeSec = Double.NaN;
        private ChassisSpeeds measuredSpeeds = new ChassisSpeeds();
        private long[] visibleTagIds = new long[0];

        private int sharedSampleCount = -1;
        private boolean sampleCountMismatch;
        private long droppedPhoenixSamples;
        private long droppedTimestampSamples;

        private long visionEventSequence;
        private long rejectCount;
        private long jumpCount;
        private String lastVisionEventType = "none";
        private double lastVisionEventTimestampSeconds = Double.NaN;
        private int lastVisionEventCamera = -1;
        private Pose2d lastVisionEventMeasuredPose = new Pose2d();
        private Pose2d lastVisionEventOdometryPose = new Pose2d();
        private double lastVisionEventTranslationDeltaMeters = Double.NaN;
        private double lastVisionEventHeadingDeltaDegrees = Double.NaN;
        private int lastVisionEventTagCount;
        private long[] lastVisionEventTagIds = new long[0];
        private double lastVisionEventAmbiguity = Double.NaN;
        private double lastVisionEventAvgDistanceMeters = Double.NaN;
        private double lastVisionEventLinearStdDev = Double.NaN;
        private double lastVisionEventAngularStdDev = Double.NaN;
    }

    private static final class PendingVisionEvent {
        private final long sequence;
        private final double logTimestampSeconds;
        private final boolean enabled;
        private final String mode;
        private final double matchTime;
        private final boolean visionConnected;
        private final boolean[] cameraConnected;
        private final boolean visionDisabledOverride;
        private String type = "unknown";
        private double observationTimestampSeconds = Double.NaN;
        private int cameraIndex = -1;
        private Pose2d measuredPose = new Pose2d();
        private boolean measuredPoseSeen;
        private Pose2d odometryPose = new Pose2d();
        private boolean odometryPoseSeen;
        private double translationDeltaMeters = Double.NaN;
        private double headingDeltaDegrees = Double.NaN;
        private int tagCount;
        private long[] tagIds = new long[0];
        private double ambiguity = Double.NaN;
        private double avgDistanceMeters = Double.NaN;
        private double linearStdDev = Double.NaN;
        private double angularStdDev = Double.NaN;

        private PendingVisionEvent(long sequence, double logTimestampSeconds, State state) {
            this.sequence = sequence;
            this.logTimestampSeconds = logTimestampSeconds;
            this.enabled = state.enabled;
            this.mode = state.mode;
            this.matchTime = state.matchTime;
            this.visionConnected = state.visionConnected;
            this.cameraConnected = Arrays.copyOf(state.cameraConnected, state.cameraConnected.length);
            this.visionDisabledOverride = state.visionDisabledOverride;
        }

        private VisionEvent finish(State state) {
            if (!Double.isFinite(observationTimestampSeconds)) {
                observationTimestampSeconds = state.lastVisionEventTimestampSeconds;
            }
            if (cameraIndex < 0) {
                cameraIndex = state.lastVisionEventCamera;
            }
            if ("unknown".equals(type)) {
                type = state.lastVisionEventType;
            }
            if (!measuredPoseSeen) {
                measuredPose = state.lastVisionEventMeasuredPose;
            }
            if (!odometryPoseSeen) {
                odometryPose = state.lastVisionEventOdometryPose;
            }
            if (!Double.isFinite(translationDeltaMeters)) {
                translationDeltaMeters = state.lastVisionEventTranslationDeltaMeters;
            }
            if (!Double.isFinite(headingDeltaDegrees)) {
                headingDeltaDegrees = state.lastVisionEventHeadingDeltaDegrees;
            }
            if (!Double.isFinite(ambiguity)) {
                ambiguity = state.lastVisionEventAmbiguity;
            }
            if (!Double.isFinite(avgDistanceMeters)) {
                avgDistanceMeters = state.lastVisionEventAvgDistanceMeters;
            }
            if (!Double.isFinite(linearStdDev)) {
                linearStdDev = state.lastVisionEventLinearStdDev;
            }
            if (!Double.isFinite(angularStdDev)) {
                angularStdDev = state.lastVisionEventAngularStdDev;
            }
            if (tagIds.length == 0) {
                tagIds = state.lastVisionEventTagIds;
            }
            if (tagCount == 0) {
                tagCount = state.lastVisionEventTagCount;
            }
            return new VisionEvent(
                    sequence,
                    type,
                    logTimestampSeconds,
                    observationTimestampSeconds,
                    enabled,
                    mode,
                    matchTime,
                    visionConnected,
                    Arrays.copyOf(cameraConnected, cameraConnected.length),
                    visionDisabledOverride,
                    measuredPose,
                    odometryPose,
                    translationDeltaMeters,
                    headingDeltaDegrees,
                    cameraIndex,
                    tagCount,
                    tagIds,
                    ambiguity,
                    avgDistanceMeters,
                    linearStdDev,
                    angularStdDev);
        }
    }

    private record VisionEvent(
            long sequence,
            String type,
            double logTimestampSeconds,
            double observationTimestampSeconds,
            boolean enabled,
            String mode,
            double matchTime,
            boolean visionConnected,
            boolean[] cameraConnected,
            boolean visionDisabledOverride,
            Pose2d measuredPose,
            Pose2d odometryPose,
            double translationDeltaMeters,
            double headingDeltaDegrees,
            int cameraIndex,
            int tagCount,
            long[] tagIds,
            double ambiguity,
            double avgDistanceMeters,
            double linearStdDev,
            double angularStdDev) {
        private String formatLine() {
            return String.format(
                    Locale.US,
                    "seq=%d type=%s logT=%.3f obsT=%.3f latency=%.0fms cam=%d enabled=%s mode=%s matchTime=%.1f vision=%s cam0=%s cam1=%s disabled=%s dPos=%.2fm dYaw=%.1fdeg tags=%d%s amb=%.3f avgDist=%.2fm std=(%.3f,%.3f) measured=%s odom=%s",
                    sequence,
                    type,
                    logTimestampSeconds,
                    observationTimestampSeconds,
                    Double.isFinite(observationTimestampSeconds)
                            ? (logTimestampSeconds - observationTimestampSeconds) * 1000.0
                            : Double.NaN,
                    cameraIndex,
                    enabled,
                    mode,
                    matchTime,
                    visionConnected,
                    cameraConnected.length > 0 ? cameraConnected[0] : false,
                    cameraConnected.length > 1 ? cameraConnected[1] : false,
                    visionDisabledOverride,
                    translationDeltaMeters,
                    headingDeltaDegrees,
                    tagCount,
                    Arrays.toString(tagIds),
                    ambiguity,
                    avgDistanceMeters,
                    linearStdDev,
                    angularStdDev,
                    formatPose(measuredPose),
                    formatPose(odometryPose));
        }
    }

    private static final class LogAnalysis {
        private final Path path;
        private final Map<Integer, EntryInfo> entries;
        private final DurationTracker enabledTracker = new DurationTracker();
        private final DurationTracker visionDisconnectedTracker = new DurationTracker();
        private final DurationTracker[] cameraDisconnectedTrackers =
                new DurationTracker[] { new DurationTracker(), new DurationTracker() };
        private final DurationTracker gyroDisconnectedTracker = new DurationTracker();
        private final DurationTracker visionDisabledTracker = new DurationTracker();
        private final DurationTracker unifiedValidTracker = new DurationTracker();
        private final DurationTracker rawValidTracker = new DurationTracker();
        private final DurationTracker unifiedInvalidWhileVisionConnectedTracker = new DurationTracker();
        private final DurationTracker visibleTagsPresentTracker = new DurationTracker();
        private final DurationTracker visibleTagsButUnifiedInvalidTracker = new DurationTracker();
        private final DurationTracker sharedSampleZeroTracker = new DurationTracker();
        private final DurationTracker sampleMismatchTracker = new DurationTracker();

        private final Distribution odometryUpdateGapSec = new Distribution();
        private final Distribution odometryStepDistanceMeters = new Distribution();
        private final Distribution odometryStepHeadingDegrees = new Distribution();
        private final Distribution odometryImpliedLinearSpeed = new Distribution();
        private final Distribution odometryImpliedAngularSpeedDegPerSec = new Distribution();
        private final Distribution unifiedUpdateGapSec = new Distribution();
        private final Distribution rawUpdateGapSec = new Distribution();
        private final Distribution acceptedTranslationDelta = new Distribution();
        private final Distribution acceptedHeadingDelta = new Distribution();
        private final Distribution rawTranslationDelta = new Distribution();
        private final Distribution rawHeadingDelta = new Distribution();
        private final Distribution sharedSampleCounts = new Distribution();

        private final List<String> transitionEvents = new ArrayList<>();
        private final List<String> connectionEvents = new ArrayList<>();
        private final List<String> anomalies = new ArrayList<>();
        private final List<VisionEvent> visionEvents = new ArrayList<>();
        private final TopSamples largestAcceptedPoseDeltas = new TopSamples(8);
        private final TopSamples largestRawPoseDeltas = new TopSamples(8);
        private final TopSamples largestOdometrySpeedSteps = new TopSamples(8);

        private long firstTimestampMicros;
        private long lastTimestampMicros;
        private int visionConnectedTransitions;
        private final int[] cameraConnectedTransitions = new int[2];
        private int gyroConnectedTransitions;
        private long maxDroppedPhoenixSamples;
        private long maxDroppedTimestampSamples;
        private int maxVisibleTagCount;
        private long finalRejectCount;
        private long finalJumpCount;

        private Pose2d lastOdomPose;
        private double lastOdomPoseTimeSec = Double.NaN;
        private double lastUnifiedPoseTimeSec = Double.NaN;
        private double lastRawPoseTimeSec = Double.NaN;

        private LogAnalysis(Path path, Map<Integer, EntryInfo> entries) {
            this.path = path.toAbsolutePath();
            this.entries = entries;
        }

        private void observeOdometryPose(double tsSec, State state) {
            if (Double.isFinite(lastOdomPoseTimeSec)) {
                double dt = tsSec - lastOdomPoseTimeSec;
                double distance = translationDelta(lastOdomPose, state.odometryPose);
                double headingDeg = headingDeltaDegrees(lastOdomPose.getRotation(), state.odometryPose.getRotation());
                double impliedLinear = dt > 1e-9 ? distance / dt : Double.NaN;
                double impliedAngular = dt > 1e-9 ? headingDeg / dt : Double.NaN;
                odometryUpdateGapSec.add(dt);
                odometryStepDistanceMeters.add(distance);
                odometryStepHeadingDegrees.add(headingDeg);
                odometryImpliedLinearSpeed.add(impliedLinear);
                odometryImpliedAngularSpeedDegPerSec.add(impliedAngular);

                if (state.enabled) {
                    double measuredLinear = Math.hypot(
                            state.measuredSpeeds.vxMetersPerSecond,
                            state.measuredSpeeds.vyMetersPerSecond);
                    if ((Double.isFinite(impliedLinear) && impliedLinear > 6.5)
                            || distance > Math.max(0.25, measuredLinear * Math.max(dt, 0.02) + 0.18)
                            || (Double.isFinite(impliedAngular) && impliedAngular > 800.0 && headingDeg > 12.0)) {
                        largestOdometrySpeedSteps.offer(
                                impliedLinear,
                                String.format(
                                        Locale.US,
                                        "t=%.3f dt=%.4fs dist=%.3fm heading=%.1fdeg implied=(%.2fm/s,%.1fdeg/s) measured=(%.2f,%.1fdeg/s) mode=%s matchTime=%.1f enabled=%s vision=%s cam0=%s cam1=%s shared=%d mismatch=%s odom=%s",
                                        tsSec,
                                        dt,
                                        distance,
                                        headingDeg,
                                        impliedLinear,
                                        impliedAngular,
                                        measuredLinear,
                                        Math.toDegrees(state.measuredSpeeds.omegaRadiansPerSecond),
                                        state.mode,
                                        state.matchTime,
                                        state.enabled,
                                        state.visionConnected,
                                        state.cameraConnected[0],
                                        state.cameraConnected[1],
                                        state.sharedSampleCount,
                                        state.sampleCountMismatch,
                                        formatPose(state.odometryPose)));
                    }
                }
            }
            lastOdomPose = state.odometryPose;
            lastOdomPoseTimeSec = tsSec;
        }

        private void observeUnifiedPose(double tsSec, State state, boolean raw) {
            Pose2d pose = raw ? state.rawPose : state.unifiedPose;
            double translationDelta = translationDelta(pose, state.odometryPose);
            double headingDelta = headingDeltaDegrees(pose.getRotation(), state.odometryPose.getRotation());
            boolean poseLooksLikeDefaultZero = Math.abs(pose.getX()) < 1e-9
                    && Math.abs(pose.getY()) < 1e-9
                    && Math.abs(pose.getRotation().getRadians()) < 1e-9;
            if (poseLooksLikeDefaultZero && translationDelta > 1.0) {
                return;
            }
            if (raw) {
                rawTranslationDelta.add(translationDelta);
                rawHeadingDelta.add(headingDelta);
                if (Double.isFinite(lastRawPoseTimeSec)) {
                    rawUpdateGapSec.add(tsSec - lastRawPoseTimeSec);
                }
                lastRawPoseTimeSec = tsSec;
                largestRawPoseDeltas.offer(
                        translationDelta,
                        String.format(
                                Locale.US,
                                "t=%.3f delta=%.3fm heading=%.1fdeg mode=%s matchTime=%.1f enabled=%s vision=%s cam0=%s cam1=%s tags=%s raw=%s odom=%s",
                                tsSec,
                                translationDelta,
                                headingDelta,
                                state.mode,
                                state.matchTime,
                                state.enabled,
                                state.visionConnected,
                                state.cameraConnected[0],
                                state.cameraConnected[1],
                                formatTagIds(state.visibleTagIds),
                                formatPose(pose),
                                formatPose(state.odometryPose)));
            } else {
                acceptedTranslationDelta.add(translationDelta);
                acceptedHeadingDelta.add(headingDelta);
                if (Double.isFinite(lastUnifiedPoseTimeSec)) {
                    unifiedUpdateGapSec.add(tsSec - lastUnifiedPoseTimeSec);
                }
                lastUnifiedPoseTimeSec = tsSec;
                largestAcceptedPoseDeltas.offer(
                        translationDelta,
                        String.format(
                                Locale.US,
                                "t=%.3f delta=%.3fm heading=%.1fdeg mode=%s matchTime=%.1f enabled=%s vision=%s cam0=%s cam1=%s tags=%s unified=%s odom=%s",
                                tsSec,
                                translationDelta,
                                headingDelta,
                                state.mode,
                                state.matchTime,
                                state.enabled,
                                state.visionConnected,
                                state.cameraConnected[0],
                                state.cameraConnected[1],
                                formatTagIds(state.visibleTagIds),
                                formatPose(pose),
                                formatPose(state.odometryPose)));
            }
        }

        private void advanceWindow(long windowStartMicros, long nextWindowMicros, State state) {
            if (nextWindowMicros <= windowStartMicros) {
                return;
            }
            double dtSec = (nextWindowMicros - windowStartMicros) / 1_000_000.0;
            boolean enabled = state.enabled;
            enabledTracker.step(enabled, dtSec);
            visionDisconnectedTracker.step(enabled && !state.visionConnected, dtSec);
            cameraDisconnectedTrackers[0].step(enabled && !state.cameraConnected[0], dtSec);
            cameraDisconnectedTrackers[1].step(enabled && !state.cameraConnected[1], dtSec);
            gyroDisconnectedTracker.step(enabled && !state.gyroConnected, dtSec);
            visionDisabledTracker.step(enabled && state.visionDisabledOverride, dtSec);
            unifiedValidTracker.step(enabled && state.unifiedPoseValid, dtSec);
            rawValidTracker.step(enabled && state.rawPoseValid, dtSec);
            unifiedInvalidWhileVisionConnectedTracker.step(enabled && state.visionConnected && !state.unifiedPoseValid, dtSec);
            visibleTagsPresentTracker.step(enabled && state.visibleTagIds.length > 0, dtSec);
            visibleTagsButUnifiedInvalidTracker.step(enabled && state.visibleTagIds.length > 0 && !state.unifiedPoseValid, dtSec);
            sharedSampleZeroTracker.step(enabled && state.sharedSampleCount == 0, dtSec);
            sampleMismatchTracker.step(enabled && state.sampleCountMismatch, dtSec);
        }

        private void finalizeSummary(State state) {
            finalRejectCount = state.rejectCount;
            finalJumpCount = state.jumpCount;
        }

        private String format() {
            long enabledRejects = visionEvents.stream().filter(event -> event.enabled() && "reject".equals(event.type())).count();
            long enabledJumps = visionEvents.stream().filter(event -> event.enabled() && "jump".equals(event.type())).count();
            long disabledRejects = visionEvents.stream().filter(event -> !event.enabled() && "reject".equals(event.type())).count();
            long disabledJumps = visionEvents.stream().filter(event -> !event.enabled() && "jump".equals(event.type())).count();
            List<String> notableEnabledEvents = visionEvents.stream()
                    .filter(VisionEvent::enabled)
                    .sorted((a, b) -> Double.compare(b.translationDeltaMeters(), a.translationDeltaMeters()))
                    .limit(12)
                    .map(VisionEvent::formatLine)
                    .toList();
            List<String> notableDisabledEvents = visionEvents.stream()
                    .filter(event -> !event.enabled())
                    .sorted((a, b) -> Double.compare(b.translationDeltaMeters(), a.translationDeltaMeters()))
                    .limit(6)
                    .map(VisionEvent::formatLine)
                    .toList();

            StringBuilder out = new StringBuilder();
            out.append("=== ").append(path.getFileName()).append(" ===\n");
            out.append("path=").append(path).append('\n');
            out.append("timeRange=")
                    .append(String.format(Locale.US, "%.3f -> %.3f sec (duration %.1fs)",
                            firstTimestampMicros / 1_000_000.0,
                            lastTimestampMicros / 1_000_000.0,
                            (lastTimestampMicros - firstTimestampMicros) / 1_000_000.0))
                    .append('\n');
            out.append("key durations while enabled\n");
            out.append("  enabled=").append(format1(enabledTracker.totalSec)).append("s\n");
            out.append("  vision disconnected=").append(format1(visionDisconnectedTracker.totalSec))
                    .append("s maxContinuous=").append(format1(visionDisconnectedTracker.maxContinuousSec))
                    .append("s transitions=").append(visionConnectedTransitions).append('\n');
            out.append("  cam0 disconnected=").append(format1(cameraDisconnectedTrackers[0].totalSec))
                    .append("s maxContinuous=").append(format1(cameraDisconnectedTrackers[0].maxContinuousSec))
                    .append("s transitions=").append(cameraConnectedTransitions[0]).append('\n');
            out.append("  cam1 disconnected=").append(format1(cameraDisconnectedTrackers[1].totalSec))
                    .append("s maxContinuous=").append(format1(cameraDisconnectedTrackers[1].maxContinuousSec))
                    .append("s transitions=").append(cameraConnectedTransitions[1]).append('\n');
            out.append("  gyro disconnected=").append(format1(gyroDisconnectedTracker.totalSec))
                    .append("s maxContinuous=").append(format1(gyroDisconnectedTracker.maxContinuousSec))
                    .append("s transitions=").append(gyroConnectedTransitions).append('\n');
            out.append("  vision override disabled=").append(format1(visionDisabledTracker.totalSec)).append("s\n");
            out.append("  unified valid=").append(format1(unifiedValidTracker.totalSec))
                    .append("s (").append(percent(unifiedValidTracker.totalSec, enabledTracker.totalSec)).append(")\n");
            out.append("  raw valid=").append(format1(rawValidTracker.totalSec))
                    .append("s (").append(percent(rawValidTracker.totalSec, enabledTracker.totalSec)).append(")\n");
            out.append("  unified invalid while vision connected=")
                    .append(format1(unifiedInvalidWhileVisionConnectedTracker.totalSec))
                    .append("s maxContinuous=")
                    .append(format1(unifiedInvalidWhileVisionConnectedTracker.maxContinuousSec))
                    .append("s\n");
            out.append("  visible tags present=").append(format1(visibleTagsPresentTracker.totalSec))
                    .append("s maxTagCount=").append(maxVisibleTagCount).append('\n');
            out.append("  visible tags but unified invalid=")
                    .append(format1(visibleTagsButUnifiedInvalidTracker.totalSec))
                    .append("s maxContinuous=")
                    .append(format1(visibleTagsButUnifiedInvalidTracker.maxContinuousSec)).append("s\n");
            out.append("  shared odometry sample count zero=").append(format1(sharedSampleZeroTracker.totalSec))
                    .append("s maxContinuous=").append(format1(sharedSampleZeroTracker.maxContinuousSec)).append("s\n");
            out.append("  odometry sample mismatch=").append(format1(sampleMismatchTracker.totalSec))
                    .append("s maxContinuous=").append(format1(sampleMismatchTracker.maxContinuousSec)).append("s\n");
            out.append('\n');

            out.append("odometry update stats\n");
            out.append("  update gap sec mean/p95/max = ")
                    .append(odometryUpdateGapSec.summary()).append('\n');
            out.append("  step distance m mean/p95/max = ")
                    .append(odometryStepDistanceMeters.summary()).append('\n');
            out.append("  step heading deg mean/p95/max = ")
                    .append(odometryStepHeadingDegrees.summary()).append('\n');
            out.append("  implied linear speed m/s mean/p95/max = ")
                    .append(odometryImpliedLinearSpeed.summary()).append('\n');
            out.append("  implied angular speed deg/s mean/p95/max = ")
                    .append(odometryImpliedAngularSpeedDegPerSec.summary()).append('\n');
            out.append("  shared sample count mean/p95/max = ")
                    .append(sharedSampleCounts.summary()).append('\n');
            out.append("  dropped phoenix samples max=").append(maxDroppedPhoenixSamples)
                    .append(" dropped timestamp samples max=").append(maxDroppedTimestampSamples).append('\n');
            out.append('\n');

            out.append("vision pose delta stats (pose vs current fused odometry at acceptance time)\n");
            out.append("  accepted translation delta m mean/p95/max = ")
                    .append(acceptedTranslationDelta.summary()).append('\n');
            out.append("  accepted heading delta deg mean/p95/max = ")
                    .append(acceptedHeadingDelta.summary()).append('\n');
            out.append("  accepted update gap sec mean/p95/max = ")
                    .append(unifiedUpdateGapSec.summary()).append('\n');
            out.append("  raw translation delta m mean/p95/max = ")
                    .append(rawTranslationDelta.summary()).append('\n');
            out.append("  raw heading delta deg mean/p95/max = ")
                    .append(rawHeadingDelta.summary()).append('\n');
            out.append("  raw update gap sec mean/p95/max = ")
                    .append(rawUpdateGapSec.summary()).append('\n');
            out.append("  reject count=").append(finalRejectCount)
                    .append(" jump count=").append(finalJumpCount)
                    .append(" reconstructed events=").append(visionEvents.size()).append('\n');
            out.append("  enabled reject/jump events=").append(enabledRejects).append('/').append(enabledJumps)
                    .append(" disabled reject/jump events=").append(disabledRejects).append('/').append(disabledJumps).append('\n');
            out.append('\n');

            out.append("largest accepted pose deltas\n");
            appendLines(out, largestAcceptedPoseDeltas.lines());
            out.append("largest raw pose deltas\n");
            appendLines(out, largestRawPoseDeltas.lines());
            out.append("largest odometry step outliers\n");
            appendLines(out, largestOdometrySpeedSteps.lines());
            out.append("notable enabled vision events\n");
            appendLines(out, notableEnabledEvents);
            out.append("notable disabled vision events\n");
            appendLines(out, notableDisabledEvents);
            out.append("connection events\n");
            appendLines(out, connectionEvents);
            out.append("state transitions\n");
            appendLines(out, transitionEvents);
            out.append("anomalies\n");
            appendLines(out, anomalies);
            return out.toString();
        }

        private void appendLines(StringBuilder out, List<String> lines) {
            if (lines.isEmpty()) {
                out.append("  <none>\n");
                return;
            }
            for (String line : lines) {
                out.append("  ").append(line).append('\n');
            }
        }
    }

    private static final class AggregateAnalysis {
        private final List<LogAnalysis> analyses;

        private AggregateAnalysis(List<LogAnalysis> analyses) {
            this.analyses = analyses;
        }

        private String format() {
            double enabledSec = analyses.stream().mapToDouble(a -> a.enabledTracker.totalSec).sum();
            double visionDiscSec = analyses.stream().mapToDouble(a -> a.visionDisconnectedTracker.totalSec).sum();
            double cam0DiscSec = analyses.stream().mapToDouble(a -> a.cameraDisconnectedTrackers[0].totalSec).sum();
            double cam1DiscSec = analyses.stream().mapToDouble(a -> a.cameraDisconnectedTrackers[1].totalSec).sum();
            double unifiedValidSec = analyses.stream().mapToDouble(a -> a.unifiedValidTracker.totalSec).sum();
            double rawValidSec = analyses.stream().mapToDouble(a -> a.rawValidTracker.totalSec).sum();
            double unifiedInvalidWhileVisionConnectedSec = analyses.stream()
                    .mapToDouble(a -> a.unifiedInvalidWhileVisionConnectedTracker.totalSec)
                    .sum();
            double visibleTagsButUnifiedInvalidSec = analyses.stream()
                    .mapToDouble(a -> a.visibleTagsButUnifiedInvalidTracker.totalSec)
                    .sum();
            double zeroSharedSec = analyses.stream().mapToDouble(a -> a.sharedSampleZeroTracker.totalSec).sum();
            double mismatchSec = analyses.stream().mapToDouble(a -> a.sampleMismatchTracker.totalSec).sum();
            long rejects = analyses.stream().mapToLong(a -> a.finalRejectCount).sum();
            long jumps = analyses.stream().mapToLong(a -> a.finalJumpCount).sum();
            long phoenixDrops = analyses.stream().mapToLong(a -> a.maxDroppedPhoenixSamples).max().orElse(0L);
            long timestampDrops = analyses.stream().mapToLong(a -> a.maxDroppedTimestampSamples).max().orElse(0L);
            double worstAccepted = analyses.stream().mapToDouble(a -> a.acceptedTranslationDelta.max()).max().orElse(Double.NaN);
            double worstRaw = analyses.stream().mapToDouble(a -> a.rawTranslationDelta.max()).max().orElse(Double.NaN);
            double worstOdomSpeed = analyses.stream().mapToDouble(a -> a.odometryImpliedLinearSpeed.max()).max().orElse(Double.NaN);
            int logsWithVisionDisconnect = (int) analyses.stream().filter(a -> a.visionDisconnectedTracker.totalSec > 0.0).count();
            int logsWithSharedZero = (int) analyses.stream().filter(a -> a.sharedSampleZeroTracker.totalSec > 0.0).count();
            int logsWithMismatches = (int) analyses.stream().filter(a -> a.sampleMismatchTracker.totalSec > 0.0).count();

            return new StringBuilder()
                    .append("  total enabled = ").append(format1(enabledSec)).append("s\n")
                    .append("  total vision disconnected = ").append(format1(visionDiscSec)).append("s in ")
                    .append(logsWithVisionDisconnect).append('/').append(analyses.size()).append(" logs\n")
                    .append("  total cam0 disconnected = ").append(format1(cam0DiscSec)).append("s\n")
                    .append("  total cam1 disconnected = ").append(format1(cam1DiscSec)).append("s\n")
                    .append("  total unified valid = ").append(format1(unifiedValidSec)).append("s (")
                    .append(percent(unifiedValidSec, enabledSec)).append(")\n")
                    .append("  total raw valid = ").append(format1(rawValidSec)).append("s (")
                    .append(percent(rawValidSec, enabledSec)).append(")\n")
                    .append("  unified invalid while vision connected = ")
                    .append(format1(unifiedInvalidWhileVisionConnectedSec)).append("s\n")
                    .append("  visible tags but unified invalid = ")
                    .append(format1(visibleTagsButUnifiedInvalidSec)).append("s\n")
                    .append("  zero shared odometry count = ").append(format1(zeroSharedSec)).append("s in ")
                    .append(logsWithSharedZero).append('/').append(analyses.size()).append(" logs\n")
                    .append("  odometry sample mismatch = ").append(format1(mismatchSec)).append("s in ")
                    .append(logsWithMismatches).append('/').append(analyses.size()).append(" logs\n")
                    .append("  reject events = ").append(rejects).append(" jump events = ").append(jumps).append('\n')
                    .append("  worst accepted delta = ").append(format2(worstAccepted)).append("m\n")
                    .append("  worst raw delta = ").append(format2(worstRaw)).append("m\n")
                    .append("  worst odometry implied speed = ").append(format2(worstOdomSpeed)).append("m/s\n")
                    .append("  max dropped phoenix samples = ").append(phoenixDrops)
                    .append(" max dropped timestamp samples = ").append(timestampDrops)
                    .toString();
        }
    }

    private static final class DurationTracker {
        private double totalSec;
        private double currentContinuousSec;
        private double maxContinuousSec;

        private void step(boolean active, double dtSec) {
            if (!(dtSec > 0.0)) {
                return;
            }
            if (active) {
                totalSec += dtSec;
                currentContinuousSec += dtSec;
                maxContinuousSec = Math.max(maxContinuousSec, currentContinuousSec);
            } else {
                currentContinuousSec = 0.0;
            }
        }
    }

    private static final class Distribution {
        private final List<Double> values = new ArrayList<>();

        private void add(double value) {
            if (Double.isFinite(value)) {
                values.add(value);
            }
        }

        private double max() {
            return values.stream().mapToDouble(Double::doubleValue).max().orElse(Double.NaN);
        }

        private String summary() {
            if (values.isEmpty()) {
                return "<none>";
            }
            List<Double> sorted = new ArrayList<>(values);
            sorted.sort(Comparator.naturalOrder());
            double mean = sorted.stream().mapToDouble(Double::doubleValue).average().orElse(Double.NaN);
            return String.format(
                    Locale.US,
                    "%.4f / %.4f / %.4f",
                    mean,
                    percentile(sorted, 0.95),
                    sorted.get(sorted.size() - 1));
        }

        private static double percentile(List<Double> sorted, double p) {
            if (sorted.isEmpty()) {
                return Double.NaN;
            }
            int index = (int) Math.ceil(p * sorted.size()) - 1;
            index = Math.max(0, Math.min(index, sorted.size() - 1));
            return sorted.get(index);
        }
    }

    private static final class TopSamples {
        private final int limit;
        private final List<RankedLine> samples = new ArrayList<>();

        private TopSamples(int limit) {
            this.limit = limit;
        }

        private void offer(double score, String line) {
            if (!Double.isFinite(score)) {
                return;
            }
            samples.add(new RankedLine(score, line));
            samples.sort((a, b) -> Double.compare(b.score, a.score));
            if (samples.size() > limit) {
                samples.remove(samples.size() - 1);
            }
        }

        private List<String> lines() {
            return samples.stream().map(sample -> sample.line).toList();
        }

        private static final class RankedLine {
            private final double score;
            private final String line;

            private RankedLine(double score, String line) {
                this.score = score;
                this.line = line;
            }
        }
    }
}
