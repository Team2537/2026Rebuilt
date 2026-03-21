package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.Test;

class ShooterHomeLogAnalyzerTest {
    private static final double CURRENT_THRESHOLD_AMPS = ShooterConstants.HOMING_CURRENT_THRESHOLD_AMPS;
    private static final long FOCUS_WINDOW_BEFORE_US = 3_000_000L;
    private static final long FOCUS_WINDOW_AFTER_US = 6_000_000L;

    @Test
    void analyzeShooterHomeTimelineFromWpiLog() throws IOException {
        String wpilogPath = System.getProperty("shooterHomeDiag.wpilog");
        if (wpilogPath == null || wpilogPath.isBlank()) {
            wpilogPath = System.getenv("SHOOTER_HOME_DIAG_WPILOG");
        }
        assumeTrue(
                wpilogPath != null && !wpilogPath.isBlank(),
                "Missing shooter-home path. Set -DshooterHomeDiag.wpilog=<path> or SHOOTER_HOME_DIAG_WPILOG=<path>.");

        Path wpilog = Path.of(wpilogPath).toAbsolutePath();
        assertTrue(Files.exists(wpilog), "WPILOG does not exist: " + wpilog);

        ShooterHomeSummary summary = analyze(wpilog);
        System.out.println(formatSummary(wpilog, summary));
    }

    private static ShooterHomeSummary analyze(Path wpilog) throws IOException {
        DataLogReader reader = new DataLogReader(wpilog.toString());
        if (!reader.isValid()) {
            throw new IllegalStateException("Invalid WPILOG file: " + wpilog);
        }

        EntryIds entryIds = new EntryIds();
        StringBuilder parseWarning = new StringBuilder();
        Iterator<DataLogRecord> startIterator = reader.iterator();
        while (true) {
            DataLogRecord record;
            try {
                if (!startIterator.hasNext()) {
                    break;
                }
                record = startIterator.next();
            } catch (RuntimeException exception) {
                parseWarning.append("start-scan terminated early: ")
                        .append(exception.getMessage());
                break;
            }
            if (!record.isStart()) {
                continue;
            }
            var start = record.getStartData();
            entryIds.capture(start.entry, start.name);
        }

        long firstTeleopUs = findFirstModeTimestampUs(wpilog, entryIds.modeEntry, "TELEOP");
        TimelineState state = new TimelineState(firstTeleopUs);

        DataLogReader dataReader = new DataLogReader(wpilog.toString());
        Iterator<DataLogRecord> dataIterator = dataReader.iterator();
        while (true) {
            DataLogRecord record;
            try {
                if (!dataIterator.hasNext()) {
                    break;
                }
                record = dataIterator.next();
            } catch (RuntimeException exception) {
                if (parseWarning.length() > 0) {
                    parseWarning.append("; ");
                }
                parseWarning.append("data-scan terminated early: ")
                        .append(exception.getMessage());
                break;
            }
            if (record.isStart() || record.isControl()) {
                continue;
            }

            int entry = record.getEntry();
            long timestampUs = record.getTimestamp();

            if (entry == entryIds.autoSelectedCommandNameEntry) {
                String newValue = record.getString();
                if (!newValue.equals(state.autoSelectedCommandName)) {
                    state.autoSelectedCommandName = newValue;
                    appendEvent(state, timestampUs, "autoSelected=" + newValue, true);
                }
                continue;
            }
            if (entry == entryIds.modeEntry) {
                String newValue = record.getString();
                if (!newValue.equals(state.mode)) {
                    state.mode = newValue;
                    appendEvent(state, timestampUs, "mode=" + newValue, true);
                }
                capturePreTeleopSnapshot(state, timestampUs);
                continue;
            }
            if (entry == entryIds.enabledEntry) {
                boolean newValue = record.getBoolean();
                if (newValue != state.enabled) {
                    state.enabled = newValue;
                    appendEvent(state, timestampUs, "enabled=" + newValue, true);
                }
                capturePreTeleopSnapshot(state, timestampUs);
                continue;
            }
            if (entry == entryIds.lastCancelAllSourceEntry) {
                String newValue = record.getString();
                if (!newValue.equals(state.lastCancelAllSource)) {
                    state.lastCancelAllSource = newValue;
                    appendEvent(state, timestampUs, "cancelAll source=" + newValue, true);
                }
                capturePreTeleopSnapshot(state, timestampUs);
                continue;
            }
            if (entry == entryIds.hoodPositionEntry) {
                state.hoodPositionDeg = Math.toDegrees(record.getDouble());
                capturePreTeleopSnapshot(state, timestampUs);
                maybeAppendHoodSnapshot(state, timestampUs, "hoodPos");
                continue;
            }
            if (entry == entryIds.targetHoodDegEntry) {
                double newValue = record.getDouble();
                boolean changed = Double.isNaN(state.targetHoodDeg) || Math.abs(newValue - state.targetHoodDeg) > 0.2;
                state.targetHoodDeg = newValue;
                capturePreTeleopSnapshot(state, timestampUs);
                if (changed) {
                    appendEvent(state, timestampUs, String.format(Locale.US, "targetHoodDeg=%.2f", newValue), false);
                }
                continue;
            }
            if (entry == entryIds.hoodAppliedVoltsEntry) {
                double newValue = record.getDouble();
                boolean changed = Double.isNaN(state.hoodAppliedVolts) || Math.abs(newValue - state.hoodAppliedVolts) > 0.25;
                state.hoodAppliedVolts = newValue;
                capturePreTeleopSnapshot(state, timestampUs);
                if (changed) {
                    appendEvent(state, timestampUs, String.format(Locale.US, "hoodVolts=%.2f", newValue), false);
                }
                continue;
            }
            if (entry == entryIds.hoodStatorCurrentEntry) {
                double newValue = record.getDouble();
                boolean crossedUp = Math.abs(state.hoodStatorCurrentAmps) <= CURRENT_THRESHOLD_AMPS
                        && Math.abs(newValue) > CURRENT_THRESHOLD_AMPS;
                boolean crossedDown = Math.abs(state.hoodStatorCurrentAmps) > CURRENT_THRESHOLD_AMPS
                        && Math.abs(newValue) <= CURRENT_THRESHOLD_AMPS;
                state.hoodStatorCurrentAmps = newValue;
                capturePreTeleopSnapshot(state, timestampUs);
                if (crossedUp || crossedDown) {
                    appendEvent(
                            state,
                            timestampUs,
                            String.format(Locale.US, "hoodCurrent=%.1fA thresholdCross=%s", newValue, crossedUp ? "UP" : "DOWN"),
                            true);
                }
                continue;
            }
            if (entry == entryIds.hoodErrorDegEntry) {
                state.hoodErrorDeg = record.getDouble();
                capturePreTeleopSnapshot(state, timestampUs);
                continue;
            }
            if (entry == entryIds.hoodAtSetpointEntry) {
                boolean newValue = record.getBoolean();
                if (newValue != state.hoodAtSetpoint) {
                    state.hoodAtSetpoint = newValue;
                    appendEvent(state, timestampUs, "hoodAtSetpoint=" + newValue, false);
                }
                capturePreTeleopSnapshot(state, timestampUs);
                continue;
            }
            if (entry == entryIds.atSetpointEntry) {
                boolean newValue = record.getBoolean();
                if (newValue != state.shooterAtSetpoint) {
                    state.shooterAtSetpoint = newValue;
                    appendEvent(state, timestampUs, "shooterAtSetpoint=" + newValue, false);
                }
                capturePreTeleopSnapshot(state, timestampUs);
                continue;
            }
            if (entry == entryIds.shooterHomeRunningEntry) {
                boolean newValue = record.getBoolean();
                if (newValue != state.shooterHomeRunning) {
                    state.shooterHomeRunning = newValue;
                    appendEvent(state, timestampUs, "ShooterHome.running=" + newValue, true);
                }
                capturePreTeleopSnapshot(state, timestampUs);
                continue;
            }
            if (entry == entryIds.shooterBackgroundRunningEntry) {
                boolean newValue = record.getBoolean();
                if (newValue != state.shooterBackgroundRunning) {
                    state.shooterBackgroundRunning = newValue;
                    appendEvent(state, timestampUs, "ShooterBackground.running=" + newValue, false);
                }
                capturePreTeleopSnapshot(state, timestampUs);
                continue;
            }
            if (entry == entryIds.lastStartedNameEntry) {
                state.lastStartedName = record.getString();
                continue;
            }
            if (entry == entryIds.lastStartedSourceEntry) {
                state.lastStartedSource = record.getString();
                continue;
            }
            if (entry == entryIds.lastStartedRunIdEntry) {
                int runId = (int) record.getInteger();
                appendEvent(
                        state,
                        timestampUs,
                        String.format(
                                Locale.US,
                                "START name=%s run=%d source=%s",
                                emptyIfNull(state.lastStartedName),
                                runId,
                                emptyIfNull(state.lastStartedSource)),
                        false);
                if ("ShooterHome".equals(state.lastStartedName)) {
                    String event = String.format(
                            Locale.US,
                            "START ShooterHome run=%d source=%s",
                            runId,
                            emptyIfNull(state.lastStartedSource));
                    appendEvent(state, timestampUs, event, true);
                    state.shooterHomeEvents.add(String.format(
                            Locale.US,
                            "t=%.3f START run=%d source=%s %s",
                            timestampUs / 1_000_000.0,
                            runId,
                            emptyIfNull(state.lastStartedSource),
                            state.snapshotSuffix()));
                }
                capturePreTeleopSnapshot(state, timestampUs);
                continue;
            }
            if (entry == entryIds.lastEndedNameEntry) {
                state.lastEndedName = record.getString();
                continue;
            }
            if (entry == entryIds.lastEndedSourceEntry) {
                state.lastEndedSource = record.getString();
                continue;
            }
            if (entry == entryIds.lastEndedInterruptedEntry) {
                state.lastEndedInterrupted = record.getBoolean();
                continue;
            }
            if (entry == entryIds.lastEndedDurationEntry) {
                state.lastEndedDurationSec = record.getDouble();
                continue;
            }
            if (entry == entryIds.lastEndedRunIdEntry) {
                int runId = (int) record.getInteger();
                appendEvent(
                        state,
                        timestampUs,
                        String.format(
                                Locale.US,
                                "%s name=%s run=%d source=%s duration=%.3fs",
                                state.lastEndedInterrupted ? "INTERRUPT" : "FINISH",
                                emptyIfNull(state.lastEndedName),
                                runId,
                                emptyIfNull(state.lastEndedSource),
                                state.lastEndedDurationSec),
                        false);
                if ("ShooterHome".equals(state.lastEndedName)) {
                    String event = String.format(
                            Locale.US,
                            "%s ShooterHome run=%d source=%s duration=%.3fs",
                            state.lastEndedInterrupted ? "INTERRUPT" : "FINISH",
                            runId,
                            emptyIfNull(state.lastEndedSource),
                            state.lastEndedDurationSec);
                    appendEvent(state, timestampUs, event, true);
                    state.shooterHomeEvents.add(String.format(
                            Locale.US,
                            "t=%.3f %s run=%d source=%s duration=%.3fs %s",
                            timestampUs / 1_000_000.0,
                            state.lastEndedInterrupted ? "INTERRUPT" : "FINISH",
                            runId,
                            emptyIfNull(state.lastEndedSource),
                            state.lastEndedDurationSec,
                            state.snapshotSuffix()));
                }
                capturePreTeleopSnapshot(state, timestampUs);
            }
        }

        return new ShooterHomeSummary(
                entryIds.matchedNames,
                state.autoSelectedCommandName,
                firstTeleopUs,
                state.preTeleopSnapshot,
                state.shooterHomeEvents,
                state.focusEvents,
                parseWarning.toString());
    }

    private static long findFirstModeTimestampUs(Path wpilog, int modeEntry, String desiredMode) throws IOException {
        if (modeEntry < 0) {
            return Long.MIN_VALUE;
        }
        DataLogReader reader = new DataLogReader(wpilog.toString());
        Iterator<DataLogRecord> iterator = reader.iterator();
        while (true) {
            DataLogRecord record;
            try {
                if (!iterator.hasNext()) {
                    break;
                }
                record = iterator.next();
            } catch (RuntimeException exception) {
                break;
            }
            if (record.isStart() || record.isControl() || record.getEntry() != modeEntry) {
                continue;
            }
            if (desiredMode.equals(record.getString())) {
                return record.getTimestamp();
            }
        }
        return Long.MIN_VALUE;
    }

    private static void capturePreTeleopSnapshot(TimelineState state, long timestampUs) {
        if (state.firstTeleopUs == Long.MIN_VALUE || timestampUs > state.firstTeleopUs) {
            return;
        }
        state.preTeleopSnapshot = String.format(
                Locale.US,
                "t=%.3f mode=%s enabled=%s cancelAll=%s %s",
                timestampUs / 1_000_000.0,
                emptyIfNull(state.mode),
                state.enabled,
                emptyIfNull(state.lastCancelAllSource),
                state.snapshotSuffix());
    }

    private static void maybeAppendHoodSnapshot(TimelineState state, long timestampUs, String label) {
        if (!isWithinFocusWindow(timestampUs, state.firstTeleopUs)) {
            return;
        }
        if (state.lastSnapshotEventUs != Long.MIN_VALUE && timestampUs - state.lastSnapshotEventUs < 250_000L) {
            return;
        }
        state.lastSnapshotEventUs = timestampUs;
        appendEvent(state, timestampUs, label, false);
    }

    private static void appendEvent(TimelineState state, long timestampUs, String event, boolean force) {
        if (!force && !isWithinFocusWindow(timestampUs, state.firstTeleopUs)) {
            return;
        }
        if (state.focusEvents.size() >= 300) {
            return;
        }
        state.focusEvents.add(String.format(
                Locale.US,
                "t=%.3f %s %s",
                timestampUs / 1_000_000.0,
                event,
                state.snapshotSuffix()));
    }

    private static boolean isWithinFocusWindow(long timestampUs, long teleopUs) {
        if (teleopUs == Long.MIN_VALUE) {
            return true;
        }
        return timestampUs >= teleopUs - FOCUS_WINDOW_BEFORE_US && timestampUs <= teleopUs + FOCUS_WINDOW_AFTER_US;
    }

    private static String emptyIfNull(String value) {
        return value == null ? "" : value;
    }

    private static String formatSummary(Path wpilog, ShooterHomeSummary summary) {
        StringBuilder builder = new StringBuilder();
        builder.append("Shooter home log summary").append(System.lineSeparator());
        builder.append("wpilog=").append(wpilog.toAbsolutePath()).append(System.lineSeparator());
        builder.append("autoSelected=").append(summary.autoSelectedCommandName).append(System.lineSeparator());
        builder.append("firstTeleopSec=")
                .append(summary.firstTeleopUs == Long.MIN_VALUE
                        ? "<missing>"
                        : String.format(Locale.US, "%.3f", summary.firstTeleopUs / 1_000_000.0))
                .append(System.lineSeparator());
        builder.append("preTeleopSnapshot=").append(summary.preTeleopSnapshot).append(System.lineSeparator());
        builder.append("parseWarning=")
                .append(summary.parseWarning == null || summary.parseWarning.isBlank() ? "<none>" : summary.parseWarning)
                .append(System.lineSeparator());
        builder.append("matchedKeys").append(System.lineSeparator());
        for (String name : summary.matchedNames) {
            builder.append("  ").append(name).append(System.lineSeparator());
        }
        builder.append("shooterHomeEvents").append(System.lineSeparator());
        if (summary.shooterHomeEvents.isEmpty()) {
            builder.append("  (none)").append(System.lineSeparator());
        } else {
            for (String event : summary.shooterHomeEvents) {
                builder.append("  ").append(event).append(System.lineSeparator());
            }
        }
        builder.append("focusTimeline").append(System.lineSeparator());
        if (summary.focusEvents.isEmpty()) {
            builder.append("  (none)").append(System.lineSeparator());
        } else {
            for (String event : summary.focusEvents) {
                builder.append("  ").append(event).append(System.lineSeparator());
            }
        }
        return builder.toString();
    }

    private record ShooterHomeSummary(
            List<String> matchedNames,
            String autoSelectedCommandName,
            long firstTeleopUs,
            String preTeleopSnapshot,
            List<String> shooterHomeEvents,
            List<String> focusEvents,
            String parseWarning) {}

    private static final class TimelineState {
        private final long firstTeleopUs;
        private final List<String> shooterHomeEvents = new ArrayList<>();
        private final List<String> focusEvents = new ArrayList<>();

        private String autoSelectedCommandName = "";
        private String mode = "";
        private boolean enabled = false;
        private String lastCancelAllSource = "";
        private double hoodPositionDeg = Double.NaN;
        private double targetHoodDeg = Double.NaN;
        private double hoodAppliedVolts = Double.NaN;
        private double hoodStatorCurrentAmps = 0.0;
        private double hoodErrorDeg = Double.NaN;
        private boolean hoodAtSetpoint = false;
        private boolean shooterAtSetpoint = false;
        private boolean shooterHomeRunning = false;
        private boolean shooterBackgroundRunning = false;
        private String lastStartedName = "";
        private String lastStartedSource = "";
        private String lastEndedName = "";
        private String lastEndedSource = "";
        private boolean lastEndedInterrupted = false;
        private double lastEndedDurationSec = 0.0;
        private String preTeleopSnapshot = "<none>";
        private long lastSnapshotEventUs = Long.MIN_VALUE;

        private TimelineState(long firstTeleopUs) {
            this.firstTeleopUs = firstTeleopUs;
        }

        private String snapshotSuffix() {
            return String.format(
                    Locale.US,
                    "mode=%s enabled=%s hoodPosDeg=%.2f targetDeg=%.2f hoodErrDeg=%.2f hoodVolts=%.2f hoodI=%.1f homeRunning=%s bgRunning=%s hoodAtSetpoint=%s shooterAtSetpoint=%s",
                    emptyIfNull(mode),
                    enabled,
                    hoodPositionDeg,
                    targetHoodDeg,
                    hoodErrorDeg,
                    hoodAppliedVolts,
                    hoodStatorCurrentAmps,
                    shooterHomeRunning,
                    shooterBackgroundRunning,
                    hoodAtSetpoint,
                    shooterAtSetpoint);
        }
    }

    private static final class EntryIds {
        private int autoSelectedCommandNameEntry = -1;
        private int modeEntry = -1;
        private int enabledEntry = -1;
        private int lastCancelAllSourceEntry = -1;
        private int hoodPositionEntry = -1;
        private int targetHoodDegEntry = -1;
        private int hoodAppliedVoltsEntry = -1;
        private int hoodStatorCurrentEntry = -1;
        private int hoodErrorDegEntry = -1;
        private int hoodAtSetpointEntry = -1;
        private int atSetpointEntry = -1;
        private int shooterHomeRunningEntry = -1;
        private int shooterBackgroundRunningEntry = -1;
        private int lastStartedNameEntry = -1;
        private int lastStartedSourceEntry = -1;
        private int lastStartedRunIdEntry = -1;
        private int lastEndedNameEntry = -1;
        private int lastEndedSourceEntry = -1;
        private int lastEndedRunIdEntry = -1;
        private int lastEndedInterruptedEntry = -1;
        private int lastEndedDurationEntry = -1;
        private final List<String> matchedNames = new ArrayList<>();

        private void capture(int entry, String name) {
            String lower = name.toLowerCase(Locale.ROOT);
            if (lower.contains("auto/selector/selectedcommandname")) {
                autoSelectedCommandNameEntry = entry;
                matchedNames.add(name);
            } else if (lower.contains("robotstate/mode")) {
                modeEntry = entry;
                matchedNames.add(name);
            } else if (lower.contains("robotstate/enabled")) {
                enabledEntry = entry;
                matchedNames.add(name);
            } else if (lower.contains("commands/lastcancelallsource")) {
                lastCancelAllSourceEntry = entry;
                matchedNames.add(name);
            } else if (lower.contains("shooter/hoodpositionrad")) {
                hoodPositionEntry = entry;
                matchedNames.add(name);
            } else if (lower.contains("shooter/targethooddeg")) {
                targetHoodDegEntry = entry;
                matchedNames.add(name);
            } else if (lower.contains("shooter/hoodappliedvolts")) {
                hoodAppliedVoltsEntry = entry;
                matchedNames.add(name);
            } else if (lower.contains("shooter/hoodstatorcurrentamps")) {
                hoodStatorCurrentEntry = entry;
                matchedNames.add(name);
            } else if (lower.contains("shooter/readiness/hoodangleerrordeg")) {
                hoodErrorDegEntry = entry;
                matchedNames.add(name);
            } else if (lower.contains("shooter/readiness/hoodangleatsetpoint")) {
                hoodAtSetpointEntry = entry;
                matchedNames.add(name);
            } else if (lower.contains("shooter/atsetpoint")) {
                atSetpointEntry = entry;
                matchedNames.add(name);
            } else if (lower.equals("commands/shooterhome")) {
                shooterHomeRunningEntry = entry;
                matchedNames.add(name);
            } else if (lower.equals("commands/shooterbackground")) {
                shooterBackgroundRunningEntry = entry;
                matchedNames.add(name);
            } else if (lower.contains("commands/laststarted/name")) {
                lastStartedNameEntry = entry;
                matchedNames.add(name);
            } else if (lower.contains("commands/laststarted/source")) {
                lastStartedSourceEntry = entry;
                matchedNames.add(name);
            } else if (lower.contains("commands/laststarted/runid")) {
                lastStartedRunIdEntry = entry;
                matchedNames.add(name);
            } else if (lower.contains("commands/lastended/name")) {
                lastEndedNameEntry = entry;
                matchedNames.add(name);
            } else if (lower.contains("commands/lastended/source")) {
                lastEndedSourceEntry = entry;
                matchedNames.add(name);
            } else if (lower.contains("commands/lastended/runid")) {
                lastEndedRunIdEntry = entry;
                matchedNames.add(name);
            } else if (lower.contains("commands/lastended/interrupted")) {
                lastEndedInterruptedEntry = entry;
                matchedNames.add(name);
            } else if (lower.contains("commands/lastended/durationsec")) {
                lastEndedDurationEntry = entry;
                matchedNames.add(name);
            }
        }
    }
}
