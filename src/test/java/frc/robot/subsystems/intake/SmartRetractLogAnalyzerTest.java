package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertFalse;
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

class SmartRetractLogAnalyzerTest {
    @Test
    void analyzeSmartRetractTimelineFromWpiLog() throws IOException {
        String wpilogPath = System.getProperty("smartRetractDiag.wpilog");
        if (wpilogPath == null || wpilogPath.isBlank()) {
            wpilogPath = System.getenv("SMART_RETRACT_DIAG_WPILOG");
        }
        assumeTrue(
                wpilogPath != null && !wpilogPath.isBlank(),
                "Missing smart retract path. Set -DsmartRetractDiag.wpilog=<path> or SMART_RETRACT_DIAG_WPILOG=<path>.");

        Path wpilog = Path.of(wpilogPath).toAbsolutePath();
        assertTrue(Files.exists(wpilog), "WPILOG does not exist: " + wpilog);

        SmartRetractLogSummary summary = analyze(wpilog);
        System.out.println(formatSummary(wpilog, summary));

        assertFalse(summary.sawDisabledOnly, "Smart retract never activated in this log.");
        assertTrue(summary.sawSessionActiveTrue, "Smart retract session never became active.");
        assertTrue(summary.sawFeedLatchedTrue, "Smart retract never latched feed true.");
        assertTrue(
                summary.minCommandedTargetRot <= IntakeConstants.EXTENDED_POSITION_ROT
                        - IntakeConstants.smartRetractStepRot() + 1e-6,
                String.format(
                        Locale.US,
                        "Smart retract never commanded inward motion. minTarget=%.3f",
                        summary.minCommandedTargetRot));
        assertTrue(
                summary.maxCommandedTargetRot <= IntakeConstants.EXTENDED_POSITION_ROT + 1e-6,
                String.format(
                        Locale.US,
                        "Smart retract commanded above allowed extended target. maxTarget=%.3f",
                        summary.maxCommandedTargetRot));
        assertTrue(
                summary.minCommandedTargetRot >= IntakeConstants.SMART_RETRACT_RETRACTED_POSITION_ROT - 1e-6,
                String.format(
                        Locale.US,
                        "Smart retract commanded below allowed retract target. minTarget=%.3f",
                        summary.minCommandedTargetRot));
    }

    private static SmartRetractLogSummary analyze(Path wpilog) throws IOException {
        DataLogReader reader = new DataLogReader(wpilog.toString());
        if (!reader.isValid()) {
            throw new IllegalStateException("Invalid WPILOG file: " + wpilog);
        }

        EntryIds entryIds = new EntryIds();
        for (DataLogRecord record : reader) {
            if (!record.isStart()) {
                continue;
            }
            var start = record.getStartData();
            entryIds.capture(start.entry, start.name);
        }

        TimelineState state = new TimelineState();
        Iterator<DataLogRecord> dataIterator = reader.iterator();
        while (dataIterator.hasNext()) {
            DataLogRecord record = dataIterator.next();
            if (record.isStart() || record.isControl()) {
                continue;
            }

            int entry = record.getEntry();
            long timestampUs = record.getTimestamp();

            if (entry == entryIds.selectedModeEntry) {
                String newValue = record.getString();
                if (!newValue.equals(state.selectedMode)) {
                    state.selectedMode = newValue;
                    state.sawDisabledOnly &= "DISABLED".equals(newValue);
                    appendEvent(state, timestampUs, "selectedMode=" + newValue);
                }
                continue;
            }
            if (entry == entryIds.sessionModeEntry) {
                String newValue = record.getString();
                if (!newValue.equals(state.sessionMode)) {
                    state.sessionMode = newValue;
                    state.sawDisabledOnly &= "DISABLED".equals(newValue);
                    appendEvent(state, timestampUs, "sessionMode=" + newValue);
                }
                continue;
            }
            if (entry == entryIds.sessionActiveEntry) {
                boolean newValue = record.getBoolean();
                if (newValue != state.sessionActive) {
                    state.sessionActive = newValue;
                    state.sawSessionActiveTrue |= newValue;
                    appendEvent(state, timestampUs, "sessionActive=" + newValue);
                }
                continue;
            }
            if (entry == entryIds.feedLatchedEntry) {
                boolean newValue = record.getBoolean();
                if (newValue != state.feedLatched) {
                    state.feedLatched = newValue;
                    state.sawFeedLatchedTrue |= newValue;
                    appendEvent(state, timestampUs, "feedLatched=" + newValue);
                }
                continue;
            }
            if (entry == entryIds.commandedTargetEntry) {
                double newValue = record.getDouble();
                state.minCommandedTargetRot = Math.min(state.minCommandedTargetRot, newValue);
                state.maxCommandedTargetRot = Math.max(state.maxCommandedTargetRot, newValue);
                if (Double.isNaN(state.commandedTargetRot)
                        || Math.abs(newValue - state.commandedTargetRot) > 0.15) {
                    state.commandedTargetRot = newValue;
                    appendEvent(state, timestampUs, String.format(Locale.US, "commandedTarget=%.3f", newValue));
                } else {
                    state.commandedTargetRot = newValue;
                }
                continue;
            }
            if (entry == entryIds.leftPositionEntry) {
                double newValue = record.getDouble() / (2.0 * Math.PI);
                if (Double.isNaN(state.leftPositionRot) || Math.abs(newValue - state.leftPositionRot) > 0.4) {
                    state.leftPositionRot = newValue;
                } else {
                    state.leftPositionRot = newValue;
                }
                continue;
            }
            if (entry == entryIds.signalCurrentRawEntry) {
                double newValue = record.getDouble();
                if (Double.isNaN(state.signalCurrentRawAmps) || Math.abs(newValue - state.signalCurrentRawAmps) > 1.0) {
                    state.signalCurrentRawAmps = newValue;
                } else {
                    state.signalCurrentRawAmps = newValue;
                }
                continue;
            }
            if (entry == entryIds.signalCurrentFilteredEntry) {
                state.signalCurrentFilteredAmps = record.getDouble();
                continue;
            }
            if (entry == entryIds.signalBaselineEntry) {
                state.signalBaselineAmps = record.getDouble();
                continue;
            }
            if (entry == entryIds.nibbleBackoffActiveEntry) {
                boolean newValue = record.getBoolean();
                if (newValue != state.nibbleBackoffActive) {
                    state.nibbleBackoffActive = newValue;
                    appendEvent(state, timestampUs, "nibbleBackoffActive=" + newValue);
                }
                continue;
            }
            if (entry == entryIds.nibbleSpikeCyclesEntry) {
                int newValue = (int) Math.round(record.getDouble());
                if (newValue != state.nibbleSpikeCycles) {
                    state.nibbleSpikeCycles = newValue;
                    appendEvent(state, timestampUs, "nibbleSpikeCycles=" + newValue);
                }
                continue;
            }
            if (entry == entryIds.feedTrueCyclesEntry) {
                int newValue = (int) Math.round(record.getDouble());
                if (newValue != state.feedTrueCycles && newValue >= 1) {
                    state.feedTrueCycles = newValue;
                    appendEvent(state, timestampUs, "feedTrueCycles=" + newValue);
                } else {
                    state.feedTrueCycles = newValue;
                }
                continue;
            }
            if (entry == entryIds.feedFalseCyclesEntry) {
                int newValue = (int) Math.round(record.getDouble());
                if (newValue != state.feedFalseCycles && newValue >= 1) {
                    state.feedFalseCycles = newValue;
                    appendEvent(state, timestampUs, "feedFalseCycles=" + newValue);
                } else {
                    state.feedFalseCycles = newValue;
                }
            }
        }

        return new SmartRetractLogSummary(
                state.events,
                state.selectedMode,
                state.sessionMode,
                state.sawSessionActiveTrue,
                state.sawFeedLatchedTrue,
                state.minCommandedTargetRot,
                state.maxCommandedTargetRot,
                state.sawDisabledOnly);
    }

    private static void appendEvent(TimelineState state, long timestampUs, String event) {
        if (state.events.size() >= 250) {
            return;
        }
        state.events.add(String.format(
                Locale.US,
                "t=%.3f %s leftPos=%.2f target=%.3f rawI=%.1f filteredI=%.1f baselineI=%.1f",
                timestampUs / 1_000_000.0,
                event,
                state.leftPositionRot,
                state.commandedTargetRot,
                state.signalCurrentRawAmps,
                state.signalCurrentFilteredAmps,
                state.signalBaselineAmps));
    }

    private static String formatSummary(Path wpilog, SmartRetractLogSummary summary) {
        StringBuilder builder = new StringBuilder();
        builder.append("Smart retract log summary").append(System.lineSeparator());
        builder.append("wpilog=").append(wpilog.toAbsolutePath()).append(System.lineSeparator());
        builder.append("selectedMode=").append(summary.selectedMode).append(System.lineSeparator());
        builder.append("sessionMode=").append(summary.sessionMode).append(System.lineSeparator());
        builder.append("sawSessionActiveTrue=").append(summary.sawSessionActiveTrue).append(System.lineSeparator());
        builder.append("sawFeedLatchedTrue=").append(summary.sawFeedLatchedTrue).append(System.lineSeparator());
        builder.append(String.format(
                Locale.US,
                "targetRange=[%.3f, %.3f]%n",
                summary.minCommandedTargetRot,
                summary.maxCommandedTargetRot));
        builder.append("timeline").append(System.lineSeparator());
        for (String event : summary.events) {
            builder.append("  ").append(event).append(System.lineSeparator());
        }
        return builder.toString();
    }

    private record SmartRetractLogSummary(
            List<String> events,
            String selectedMode,
            String sessionMode,
            boolean sawSessionActiveTrue,
            boolean sawFeedLatchedTrue,
            double minCommandedTargetRot,
            double maxCommandedTargetRot,
            boolean sawDisabledOnly) {}

    private static final class TimelineState {
        private final List<String> events = new ArrayList<>();
        private String selectedMode = "";
        private String sessionMode = "";
        private boolean sessionActive = false;
        private boolean feedLatched = false;
        private double commandedTargetRot = Double.NaN;
        private double leftPositionRot = Double.NaN;
        private double signalCurrentRawAmps = Double.NaN;
        private double signalCurrentFilteredAmps = Double.NaN;
        private double signalBaselineAmps = Double.NaN;
        private boolean nibbleBackoffActive = false;
        private int nibbleSpikeCycles = 0;
        private int feedTrueCycles = 0;
        private int feedFalseCycles = 0;
        private boolean sawSessionActiveTrue = false;
        private boolean sawFeedLatchedTrue = false;
        private double minCommandedTargetRot = Double.POSITIVE_INFINITY;
        private double maxCommandedTargetRot = Double.NEGATIVE_INFINITY;
        private boolean sawDisabledOnly = true;
    }

    private static final class EntryIds {
        private int selectedModeEntry = -1;
        private int sessionModeEntry = -1;
        private int sessionActiveEntry = -1;
        private int feedLatchedEntry = -1;
        private int commandedTargetEntry = -1;
        private int leftPositionEntry = -1;
        private int signalCurrentRawEntry = -1;
        private int signalCurrentFilteredEntry = -1;
        private int signalBaselineEntry = -1;
        private int nibbleBackoffActiveEntry = -1;
        private int nibbleSpikeCyclesEntry = -1;
        private int feedTrueCyclesEntry = -1;
        private int feedFalseCyclesEntry = -1;

        private void capture(int entry, String name) {
            if (name.contains("Intake/SmartRetract/SelectedMode")) {
                selectedModeEntry = entry;
            } else if (name.contains("Intake/SmartRetract/SessionMode")) {
                sessionModeEntry = entry;
            } else if (name.contains("Intake/SmartRetract/SessionActive")) {
                sessionActiveEntry = entry;
            } else if (name.contains("Intake/SmartRetract/FeedLatched")) {
                feedLatchedEntry = entry;
            } else if (name.contains("Intake/SmartRetract/CommandedTargetRot")) {
                commandedTargetEntry = entry;
            } else if (name.contains("Intake/LeftPositionRad")) {
                leftPositionEntry = entry;
            } else if (name.contains("Intake/SmartRetract/SignalCurrentRawAmps")) {
                signalCurrentRawEntry = entry;
            } else if (name.contains("Intake/SmartRetract/SignalCurrentFilteredAmps")) {
                signalCurrentFilteredEntry = entry;
            } else if (name.contains("Intake/SmartRetract/SignalBaselineAmps")) {
                signalBaselineEntry = entry;
            } else if (name.contains("Intake/SmartRetract/NibbleBackoffActive")) {
                nibbleBackoffActiveEntry = entry;
            } else if (name.contains("Intake/SmartRetract/NibbleSpikeCycles")) {
                nibbleSpikeCyclesEntry = entry;
            } else if (name.contains("Intake/SmartRetract/FeedTrueCycles")) {
                feedTrueCyclesEntry = entry;
            } else if (name.contains("Intake/SmartRetract/FeedFalseCycles")) {
                feedFalseCyclesEntry = entry;
            }
        }
    }
}
