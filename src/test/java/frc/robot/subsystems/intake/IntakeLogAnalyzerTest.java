package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import org.junit.jupiter.api.Test;

class IntakeLogAnalyzerTest {
    @Test
    void analyzeIntakeTimelineFromWpiLog() throws IOException {
        String wpilogPath = System.getProperty("intakeDiag.wpilog");
        if (wpilogPath == null || wpilogPath.isBlank()) {
            wpilogPath = System.getenv("INTAKE_DIAG_WPILOG");
        }
        assumeTrue(
                wpilogPath != null && !wpilogPath.isBlank(),
                "Missing intake path. Set -DintakeDiag.wpilog=<path> or INTAKE_DIAG_WPILOG=<path>.");

        Path wpilog = Path.of(wpilogPath).toAbsolutePath();
        assertTrue(Files.exists(wpilog), "WPILOG does not exist: " + wpilog);

        IntakeLogSummary summary = analyze(wpilog);
        System.out.println(formatSummary(wpilog, summary));
    }

    private static IntakeLogSummary analyze(Path wpilog) throws IOException {
        DataLogReader reader = new DataLogReader(wpilog.toString());
        if (!reader.isValid()) {
            throw new IllegalStateException("Invalid WPILOG file: " + wpilog);
        }

        EntryIds entryIds = new EntryIds();
        Iterator<DataLogRecord> startIterator = reader.iterator();
        while (startIterator.hasNext()) {
            DataLogRecord record = startIterator.next();
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

            if (entry == entryIds.motionStateEntry) {
                String newValue = record.getString();
                if (!newValue.equals(state.motionState)) {
                    state.motionState = newValue;
                    summaryAppend(state, timestampUs, "motionState=" + newValue);
                }
                continue;
            }
            if (entry == entryIds.goalStateEntry) {
                String newValue = record.getString();
                if (!newValue.equals(state.goalState)) {
                    state.goalState = newValue;
                    summaryAppend(state, timestampUs, "goalState=" + newValue);
                }
                continue;
            }
            if (entry == entryIds.extendedEntry) {
                boolean newValue = record.getBoolean();
                if (newValue != state.extended) {
                    state.extended = newValue;
                    summaryAppend(state, timestampUs, "extended=" + newValue);
                }
                continue;
            }
            if (entry == entryIds.requestedExtendedEntry) {
                boolean newValue = record.getBoolean();
                if (newValue != state.requestedExtended) {
                    state.requestedExtended = newValue;
                    summaryAppend(state, timestampUs, "requestedExtended=" + newValue);
                }
                continue;
            }
            if (entry == entryIds.leftAppliedVoltsEntry) {
                double newValue = record.getDouble();
                if (Math.abs(newValue - state.leftAppliedVolts) > 0.25) {
                    state.leftAppliedVolts = newValue;
                    summaryAppend(state, timestampUs, String.format(Locale.US, "leftVolts=%.2f", newValue));
                } else {
                    state.leftAppliedVolts = newValue;
                }
                continue;
            }
            if (entry == entryIds.rightAppliedVoltsEntry) {
                double newValue = record.getDouble();
                if (Math.abs(newValue - state.rightAppliedVolts) > 0.25) {
                    state.rightAppliedVolts = newValue;
                    summaryAppend(state, timestampUs, String.format(Locale.US, "rightVolts=%.2f", newValue));
                } else {
                    state.rightAppliedVolts = newValue;
                }
                continue;
            }
            if (entry == entryIds.leftPositionEntry) {
                state.leftPositionRot = record.getDouble() / (2.0 * Math.PI);
                continue;
            }
            if (entry == entryIds.rightPositionEntry) {
                state.rightPositionRot = record.getDouble() / (2.0 * Math.PI);
                continue;
            }
            if (entry == entryIds.leftCurrentEntry) {
                double newValue = record.getDouble();
                if (crossedThreshold(state.leftCurrentAmps, newValue, IntakeConstants.HOMING_CURRENT_THRESHOLD_AMPS)) {
                    summaryAppend(state, timestampUs, String.format(Locale.US, "leftCurrent=%.1fA", newValue));
                }
                state.leftCurrentAmps = newValue;
                continue;
            }
            if (entry == entryIds.rightCurrentEntry) {
                double newValue = record.getDouble();
                if (crossedThreshold(state.rightCurrentAmps, newValue, IntakeConstants.HOMING_CURRENT_THRESHOLD_AMPS)) {
                    summaryAppend(state, timestampUs, String.format(Locale.US, "rightCurrent=%.1fA", newValue));
                }
                state.rightCurrentAmps = newValue;
                continue;
            }
            if (entry == entryIds.intakeHomeRunningEntry) {
                boolean newValue = record.getBoolean();
                if (newValue != state.intakeHomeRunning) {
                    state.intakeHomeRunning = newValue;
                    summaryAppend(state, timestampUs, "IntakeHome.running=" + newValue);
                }
                continue;
            }
            if (entry == entryIds.autoNamedIntakeHomeRunningEntry) {
                boolean newValue = record.getBoolean();
                if (newValue != state.autoNamedIntakeHomeRunning) {
                    state.autoNamedIntakeHomeRunning = newValue;
                    summaryAppend(state, timestampUs, "AutoNamed_IntakeHome.running=" + newValue);
                }
                continue;
            }
            if (entry == entryIds.autoNamedIntakeExtendRunningEntry) {
                boolean newValue = record.getBoolean();
                if (newValue != state.autoNamedIntakeExtendRunning) {
                    state.autoNamedIntakeExtendRunning = newValue;
                    summaryAppend(state, timestampUs, "AutoNamed_IntakeExtend.running=" + newValue);
                }
                continue;
            }
            if (entry == entryIds.driverIntakeHomeRunningEntry) {
                boolean newValue = record.getBoolean();
                if (newValue != state.driverIntakeHomeRunning) {
                    state.driverIntakeHomeRunning = newValue;
                    summaryAppend(state, timestampUs, "DriverIntakeHome.running=" + newValue);
                }
                continue;
            }
            if (entry == entryIds.intakeBackgroundRunningEntry) {
                boolean newValue = record.getBoolean();
                if (newValue != state.intakeBackgroundRunning) {
                    state.intakeBackgroundRunning = newValue;
                    summaryAppend(state, timestampUs, "IntakeBackground.running=" + newValue);
                }
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

            if (entry == entryIds.lastStartedRunIdEntry) {
                int runId = (int) Math.round(record.getDouble());
                summaryAppend(
                        state,
                        timestampUs,
                        String.format(
                                Locale.US,
                                "lastStarted run=%d name=%s source=%s",
                                runId,
                                emptyIfNull(state.lastStartedName),
                                emptyIfNull(state.lastStartedSource)));
                continue;
            }
            if (entry == entryIds.lastEndedRunIdEntry) {
                int runId = (int) Math.round(record.getDouble());
                summaryAppend(
                        state,
                        timestampUs,
                        String.format(
                                Locale.US,
                                "lastEnded run=%d name=%s source=%s interrupted=%s duration=%.3fs",
                                runId,
                                emptyIfNull(state.lastEndedName),
                                emptyIfNull(state.lastEndedSource),
                                state.lastEndedInterrupted,
                                state.lastEndedDurationSec));
            }
        }

        return new IntakeLogSummary(state.events, state, entryIds.matchedNames);
    }

    private static boolean crossedThreshold(double previous, double current, double threshold) {
        return Math.abs(previous) <= threshold && Math.abs(current) > threshold;
    }

    private static String emptyIfNull(String value) {
        return value == null ? "" : value;
    }

    private static void summaryAppend(TimelineState state, long timestampUs, String event) {
        if (state.events.size() >= 400) {
            return;
        }
        state.events.add(String.format(
                Locale.US,
                "t=%.3f %s leftPos=%.2f rightPos=%.2f leftI=%.1f rightI=%.1f",
                timestampUs / 1_000_000.0,
                event,
                state.leftPositionRot,
                state.rightPositionRot,
                state.leftCurrentAmps,
                state.rightCurrentAmps));
    }

    private static String formatSummary(Path wpilog, IntakeLogSummary summary) {
        StringBuilder builder = new StringBuilder();
        builder.append("Intake log summary").append(System.lineSeparator());
        builder.append("wpilog=").append(wpilog.toAbsolutePath()).append(System.lineSeparator());
        builder.append("matched keys").append(System.lineSeparator());
        for (String name : summary.matchedNames) {
            builder.append("  ").append(name).append(System.lineSeparator());
        }
        builder.append("final motionState=").append(summary.finalState.motionState).append(System.lineSeparator());
        builder.append("final goalState=").append(summary.finalState.goalState).append(System.lineSeparator());
        builder.append("final extended=").append(summary.finalState.extended).append(System.lineSeparator());
        builder.append("final requestedExtended=").append(summary.finalState.requestedExtended).append(System.lineSeparator());
        builder.append(String.format(
                Locale.US,
                "final leftPos=%.2f rightPos=%.2f leftVolts=%.2f rightVolts=%.2f leftI=%.1f rightI=%.1f%n",
                summary.finalState.leftPositionRot,
                summary.finalState.rightPositionRot,
                summary.finalState.leftAppliedVolts,
                summary.finalState.rightAppliedVolts,
                summary.finalState.leftCurrentAmps,
                summary.finalState.rightCurrentAmps));
        builder.append("timeline").append(System.lineSeparator());
        for (String event : summary.events) {
            builder.append("  ").append(event).append(System.lineSeparator());
        }
        return builder.toString();
    }

    private record IntakeLogSummary(List<String> events, TimelineState finalState, List<String> matchedNames) {}

    private static final class TimelineState {
        private final List<String> events = new ArrayList<>();
        private String motionState = "";
        private String goalState = "";
        private boolean extended = false;
        private boolean requestedExtended = false;
        private double leftAppliedVolts = 0.0;
        private double rightAppliedVolts = 0.0;
        private double leftPositionRot = Double.NaN;
        private double rightPositionRot = Double.NaN;
        private double leftCurrentAmps = 0.0;
        private double rightCurrentAmps = 0.0;
        private boolean intakeHomeRunning = false;
        private boolean autoNamedIntakeHomeRunning = false;
        private boolean autoNamedIntakeExtendRunning = false;
        private boolean driverIntakeHomeRunning = false;
        private boolean intakeBackgroundRunning = false;
        private String lastStartedName = "";
        private String lastStartedSource = "";
        private String lastEndedName = "";
        private String lastEndedSource = "";
        private boolean lastEndedInterrupted = false;
        private double lastEndedDurationSec = 0.0;
    }

    private static final class EntryIds {
        private int motionStateEntry = -1;
        private int goalStateEntry = -1;
        private int extendedEntry = -1;
        private int requestedExtendedEntry = -1;
        private int leftAppliedVoltsEntry = -1;
        private int rightAppliedVoltsEntry = -1;
        private int leftPositionEntry = -1;
        private int rightPositionEntry = -1;
        private int leftCurrentEntry = -1;
        private int rightCurrentEntry = -1;
        private int intakeHomeRunningEntry = -1;
        private int autoNamedIntakeHomeRunningEntry = -1;
        private int autoNamedIntakeExtendRunningEntry = -1;
        private int driverIntakeHomeRunningEntry = -1;
        private int intakeBackgroundRunningEntry = -1;
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
            if (name.contains("Intake") || name.contains("Commands/byName") || name.contains("Commands/last")) {
                matchedNames.add(name);
            }
            if (name.contains("Intake/MotionState")) {
                motionStateEntry = entry;
            } else if (name.contains("Intake/GoalState")) {
                goalStateEntry = entry;
            } else if (name.contains("Intake/Extended") && !name.contains("Requested")) {
                extendedEntry = entry;
            } else if (name.contains("Intake/RequestedExtended")) {
                requestedExtendedEntry = entry;
            } else if (name.contains("Intake/LeftAppliedVolts")) {
                leftAppliedVoltsEntry = entry;
            } else if (name.contains("Intake/RightAppliedVolts")) {
                rightAppliedVoltsEntry = entry;
            } else if (name.contains("Intake/LeftPositionRad")) {
                leftPositionEntry = entry;
            } else if (name.contains("Intake/RightPositionRad")) {
                rightPositionEntry = entry;
            } else if (name.contains("Intake/LeftStatorCurrentAmps")) {
                leftCurrentEntry = entry;
            } else if (name.contains("Intake/RightStatorCurrentAmps")) {
                rightCurrentEntry = entry;
            } else if (name.contains("Commands/byName/IntakeHome/running")) {
                intakeHomeRunningEntry = entry;
            } else if (name.contains("Commands/byName/AutoNamed_IntakeHome/running")) {
                autoNamedIntakeHomeRunningEntry = entry;
            } else if (name.contains("Commands/byName/AutoNamed_IntakeExtend/running")) {
                autoNamedIntakeExtendRunningEntry = entry;
            } else if (name.contains("Commands/byName/DriverIntakeHome/running")) {
                driverIntakeHomeRunningEntry = entry;
            } else if (name.contains("Commands/byName/IntakeBackground/running")) {
                intakeBackgroundRunningEntry = entry;
            } else if (name.contains("Commands/lastStarted/name")) {
                lastStartedNameEntry = entry;
            } else if (name.contains("Commands/lastStarted/source")) {
                lastStartedSourceEntry = entry;
            } else if (name.contains("Commands/lastStarted/runId")) {
                lastStartedRunIdEntry = entry;
            } else if (name.contains("Commands/lastEnded/name")) {
                lastEndedNameEntry = entry;
            } else if (name.contains("Commands/lastEnded/source")) {
                lastEndedSourceEntry = entry;
            } else if (name.contains("Commands/lastEnded/runId")) {
                lastEndedRunIdEntry = entry;
            } else if (name.contains("Commands/lastEnded/interrupted")) {
                lastEndedInterruptedEntry = entry;
            } else if (name.contains("Commands/lastEnded/durationSec")) {
                lastEndedDurationEntry = entry;
            }
        }
    }
}
