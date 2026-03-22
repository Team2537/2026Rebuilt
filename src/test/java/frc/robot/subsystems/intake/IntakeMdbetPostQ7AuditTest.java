package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.OptionalInt;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.junit.jupiter.api.Test;

class IntakeMdbetPostQ7AuditTest {
    private static final Pattern QUAL_MATCH_PATTERN = Pattern.compile("_q(\\d+)\\.wpilog$");
    private static final double RETRACTED_TARGET_ROT = IntakeConstants.RETRACTED_POSITION_ROT;
    private static final double EXTENDED_TARGET_ROT = IntakeConstants.EXTENDED_POSITION_ROT;
    private static final double POSITION_TOLERANCE_ROT = IntakeConstants.POSITION_TOLERANCE_ROT;

    @Test
    void auditMdbetPostMatch7IntakeBehavior() throws IOException {
        Path directory = Path.of("logs/mdbet").toAbsolutePath();
        assertTrue(Files.isDirectory(directory), "Missing log directory: " + directory);

        List<Path> logs;
        try (var stream = Files.list(directory)) {
            logs = stream.filter(path -> path.getFileName().toString().endsWith(".wpilog"))
                    .filter(path -> parseQualificationMatch(path).orElse(0) > 7)
                    .sorted(Comparator.comparingInt(path -> parseQualificationMatch(path).orElse(Integer.MAX_VALUE)))
                    .toList();
        }

        assertFalse(logs.isEmpty(), "No post-q7 logs found in " + directory);

        List<LogSummary> summaries = new ArrayList<>();
        for (Path log : logs) {
            summaries.add(analyze(log));
        }

        String report = formatAggregateReport(directory, summaries);
        Path out = Path.of("build/reports/intake/mdbet-postq7-intake-audit.txt").toAbsolutePath();
        Files.createDirectories(out.getParent());
        Files.writeString(out, report);

        System.out.println(report);
        System.out.println("Intake post-q7 audit written: " + out);

        long logsWithSuspiciousInterrupts = summaries.stream()
                .filter(summary -> !summary.suspiciousInterrupts.isEmpty())
                .count();

        assertTrue(logsWithSuspiciousInterrupts == 0,
                () -> "Unexpected intake command interrupts found in " + logsWithSuspiciousInterrupts + " logs.");
    }

    private static LogSummary analyze(Path wpilog) throws IOException {
        DataLogReader reader = new DataLogReader(wpilog.toString());
        if (!reader.isValid()) {
            throw new IllegalStateException("Invalid WPILOG file: " + wpilog);
        }

        Map<Integer, EntryInfo> entries = new LinkedHashMap<>();
        Iterator<DataLogRecord> startIterator = reader.iterator();
        while (true) {
            try {
                if (!startIterator.hasNext()) {
                    break;
                }
                DataLogRecord record = startIterator.next();
                if (!record.isStart()) {
                    continue;
                }
                var start = record.getStartData();
                entries.put(start.entry, new EntryInfo(start.name, start.type));
            } catch (IllegalArgumentException ignored) {
                break;
            }
        }

        EntryIds ids = resolveEntries(entries);
        AnalysisState state = new AnalysisState();
        LogSummary summary = new LogSummary(wpilog, parseQualificationMatch(wpilog).orElse(-1));
        summary.discoveredIntakeCommands.addAll(ids.runningCommandNames);

        Iterator<DataLogRecord> dataIterator = reader.iterator();
        while (true) {
            final DataLogRecord record;
            try {
                if (!dataIterator.hasNext()) {
                    break;
                }
                record = dataIterator.next();
            } catch (IllegalArgumentException ignored) {
                summary.truncated = true;
                break;
            }

            if (record.isStart() || record.isControl()) {
                continue;
            }

            long timestampUs = record.getTimestamp();
            if (state.previousRecordTsUs != Long.MIN_VALUE && timestampUs < state.previousRecordTsUs) {
                summary.truncated = true;
                break;
            }
            if (summary.firstTimestampUs != Long.MAX_VALUE
                    && timestampUs - summary.firstTimestampUs > 1_000_000_000L) {
                summary.truncated = true;
                break;
            }
            state.previousRecordTsUs = timestampUs;
            summary.firstTimestampUs = Math.min(summary.firstTimestampUs, timestampUs);
            summary.lastTimestampUs = Math.max(summary.lastTimestampUs, timestampUs);

            int entry = record.getEntry();

            if (entry == ids.enabledEntry) {
                boolean newValue = record.getBoolean();
                updateEnabledState(summary, state, timestampUs, newValue);
                continue;
            }
            if (entry == ids.modeEntry) {
                String newValue = record.getString();
                if (!newValue.equals(state.mode)) {
                    state.mode = newValue;
                    summary.modeEvents.add(String.format(Locale.US, "t=%.3f mode=%s", seconds(timestampUs), newValue));
                }
                continue;
            }
            if (entry == ids.matchTimeEntry) {
                state.matchTimeSec = record.getDouble();
                continue;
            }
            if (entry == ids.motionStateEntry) {
                String newValue = record.getString();
                if (!newValue.equals(state.motionState)) {
                    handleMotionStateTransition(summary, state, timestampUs, newValue);
                }
                continue;
            }
            if (entry == ids.goalStateEntry) {
                state.goalState = record.getString();
                continue;
            }
            if (entry == ids.requestedExtendedEntry) {
                boolean newValue = record.getBoolean();
                if (newValue != state.requestedExtended) {
                    handleRequestedExtendedTransition(summary, state, timestampUs, newValue);
                }
                continue;
            }
            if (entry == ids.extendedEntry) {
                boolean newValue = record.getBoolean();
                if (newValue != state.extended) {
                    state.extended = newValue;
                    summary.extendedStateEvents.add(String.format(
                            Locale.US,
                            "t=%.3f extended=%s requested=%s left=%.2f right=%.2f goal=%s motion=%s",
                            seconds(timestampUs),
                            newValue,
                            state.requestedExtended,
                            state.leftPositionRot,
                            state.rightPositionRot,
                            state.goalState,
                            state.motionState));
                }
                continue;
            }
            if (entry == ids.leftPositionEntry) {
                double value = Units.radiansToRotations(record.getDouble());
                updateGap(summary.leftPositionGapStats, state.lastLeftPositionTsUs, timestampUs);
                state.lastLeftPositionTsUs = timestampUs;
                state.leftPositionRot = value;
                summary.leftPositionStats.add(value);
                observeSnapshot(summary, state, timestampUs);
                continue;
            }
            if (entry == ids.rightPositionEntry) {
                double value = Units.radiansToRotations(record.getDouble());
                updateGap(summary.rightPositionGapStats, state.lastRightPositionTsUs, timestampUs);
                state.lastRightPositionTsUs = timestampUs;
                state.rightPositionRot = value;
                summary.rightPositionStats.add(value);
                observeSnapshot(summary, state, timestampUs);
                continue;
            }
            if (entry == ids.leftAppliedVoltsEntry) {
                double value = record.getDouble();
                state.leftAppliedVolts = value;
                summary.leftVoltageStats.add(value);
                if (!state.enabled) {
                    maybeCaptureDisabledActuation(summary, timestampUs, state, Math.abs(value), "leftVolts", value);
                }
                continue;
            }
            if (entry == ids.rightAppliedVoltsEntry) {
                double value = record.getDouble();
                state.rightAppliedVolts = value;
                summary.rightVoltageStats.add(value);
                if (!state.enabled) {
                    maybeCaptureDisabledActuation(summary, timestampUs, state, Math.abs(value), "rightVolts", value);
                }
                continue;
            }
            if (entry == ids.leftCurrentEntry) {
                double value = record.getDouble();
                state.leftCurrentAmps = value;
                summary.leftCurrentStats.add(value);
                if (state.homeActive) {
                    state.activeHomeWindow.maxAbsLeftCurrentAmps = Math.max(
                            state.activeHomeWindow.maxAbsLeftCurrentAmps,
                            Math.abs(value));
                }
                continue;
            }
            if (entry == ids.rightCurrentEntry) {
                double value = record.getDouble();
                state.rightCurrentAmps = value;
                summary.rightCurrentStats.add(value);
                if (state.homeActive) {
                    state.activeHomeWindow.maxAbsRightCurrentAmps = Math.max(
                            state.activeHomeWindow.maxAbsRightCurrentAmps,
                            Math.abs(value));
                }
                continue;
            }
            if (entry == ids.rollerVelocityEntry) {
                double value = record.getDouble();
                updateGap(summary.rollerVelocityGapStats, state.lastRollerVelocityTsUs, timestampUs);
                state.lastRollerVelocityTsUs = timestampUs;
                state.rollerVelocityRpm = value;
                summary.rollerVelocityStats.add(value);
                observeRoller(summary, state, timestampUs, value);
                continue;
            }
            if (entry == ids.rollerAppliedVoltsEntry) {
                double value = record.getDouble();
                state.rollerAppliedVolts = value;
                summary.rollerVoltageStats.add(value);
                if (!state.enabled) {
                    maybeCaptureDisabledActuation(summary, timestampUs, state, Math.abs(value), "rollerVolts", value);
                }
                continue;
            }
            if (entry == ids.rollerCurrentEntry) {
                double value = record.getDouble();
                state.rollerCurrentAmps = value;
                summary.rollerCurrentStats.add(value);
                continue;
            }
            if (entry == ids.positionOverlayActiveEntry) {
                state.positionOverlayActive = record.getBoolean();
                continue;
            }
            if (entry == ids.driverTriggerWiggleActiveEntry) {
                state.driverTriggerWiggleActive = record.getBoolean();
                continue;
            }
            if (entry == ids.driverTriggerWiggleTargetEntry) {
                state.driverTriggerWiggleTargetRot = record.getDouble();
                continue;
            }
            if (entry == ids.driverAgitationActiveEntry) {
                state.driverAgitationActive = record.getBoolean();
                continue;
            }
            if (entry == ids.driverAgitationTargetEntry) {
                state.driverAgitationTargetRot = record.getDouble();
                continue;
            }
            if (entry == ids.smartRetractSessionActiveEntry) {
                boolean newValue = record.getBoolean();
                state.smartRetractSessionActive = newValue;
                if (newValue) {
                    summary.smartRetractSessionCount++;
                }
                continue;
            }
            if (entry == ids.smartRetractSessionModeEntry) {
                state.smartRetractSessionMode = record.getString();
                continue;
            }
            if (entry == ids.smartRetractCommandedTargetEntry) {
                double value = record.getDouble();
                state.smartRetractCommandedTargetRot = value;
                summary.smartRetractCommandedTargetStats.add(value);
                continue;
            }
            if (entry == ids.smartRetractWiggleActiveEntry) {
                state.smartRetractWiggleActive = record.getBoolean();
                continue;
            }
            if (entry == ids.smartRetractWiggleTargetEntry) {
                state.smartRetractWiggleTargetRot = record.getDouble();
                continue;
            }
            if (entry == ids.lastStartedNameEntry) {
                state.pendingLastStartedName = record.getString();
                continue;
            }
            if (entry == ids.lastStartedSourceEntry) {
                state.pendingLastStartedSource = record.getString();
                continue;
            }
            if (entry == ids.lastStartedRequirementsEntry) {
                state.pendingLastStartedRequirements = record.getString();
                continue;
            }
            if (entry == ids.lastStartedRunIdEntry) {
                int runId = (int) Math.round(record.getDouble());
                emitStartedCommand(summary, state, timestampUs, runId);
                continue;
            }
            if (entry == ids.lastEndedNameEntry) {
                state.pendingLastEndedName = record.getString();
                continue;
            }
            if (entry == ids.lastEndedSourceEntry) {
                state.pendingLastEndedSource = record.getString();
                continue;
            }
            if (entry == ids.lastEndedInterruptedEntry) {
                state.pendingLastEndedInterrupted = record.getBoolean();
                continue;
            }
            if (entry == ids.lastEndedDurationEntry) {
                state.pendingLastEndedDurationSec = record.getDouble();
                continue;
            }
            if (entry == ids.lastEndedRunIdEntry) {
                int runId = (int) Math.round(record.getDouble());
                emitEndedCommand(summary, state, timestampUs, runId);
                continue;
            }

            String commandName = ids.commandRunningEntries.get(entry);
            if (commandName != null) {
                boolean running = record.getBoolean();
                state.runningCommands.put(commandName, running);
            }
        }

        if (state.enabled && state.enabledSinceUs != Long.MIN_VALUE) {
            summary.enabledTimeSec += (summary.lastTimestampUs - state.enabledSinceUs) / 1_000_000.0;
        }
        if (state.pendingTransition != null && !state.pendingTransition.completed) {
            double ageSec = (summary.lastTimestampUs - state.pendingTransition.startUs) / 1_000_000.0;
            if (state.enabled && !"DISABLED".equals(state.mode) && ageSec >= 0.40) {
                summary.incompleteTransitions.add(state.pendingTransition.describe(summary.lastTimestampUs));
            }
        }
        if (state.homeActive && state.activeHomeWindow != null) {
            state.activeHomeWindow.endUs = summary.lastTimestampUs;
            summary.homeWindows.add(state.activeHomeWindow);
        }

        return summary;
    }

    private static void updateEnabledState(LogSummary summary, AnalysisState state, long timestampUs, boolean enabled) {
        if (enabled == state.enabled) {
            return;
        }
        if (enabled) {
            state.enabled = true;
            state.enabledSinceUs = timestampUs;
        } else {
            if (state.enabled && state.enabledSinceUs != Long.MIN_VALUE) {
                summary.enabledTimeSec += (timestampUs - state.enabledSinceUs) / 1_000_000.0;
            }
            state.enabled = false;
            state.enabledSinceUs = Long.MIN_VALUE;
        }
        summary.modeEvents.add(String.format(Locale.US, "t=%.3f enabled=%s mode=%s", seconds(timestampUs), enabled, state.mode));
    }

    private static void handleMotionStateTransition(
            LogSummary summary,
            AnalysisState state,
            long timestampUs,
            String newMotionState) {
        if (!newMotionState.equals(state.motionState)) {
            summary.motionStateEvents.add(String.format(
                    Locale.US,
                    "t=%.3f motion=%s goal=%s requested=%s left=%.2f right=%.2f",
                    seconds(timestampUs),
                    newMotionState,
                    state.goalState,
                    state.requestedExtended,
                    state.leftPositionRot,
                    state.rightPositionRot));
        }

        boolean enteringHome = "HOMING".equals(newMotionState) && !"HOMING".equals(state.motionState);
        boolean leavingHome = "HOMING".equals(state.motionState) && !"HOMING".equals(newMotionState);
        if (enteringHome) {
            state.homeActive = true;
            state.activeHomeWindow = new HomeWindow(timestampUs);
        }
        if (leavingHome && state.homeActive && state.activeHomeWindow != null) {
            state.activeHomeWindow.endUs = timestampUs;
            summary.homeWindows.add(state.activeHomeWindow);
            state.homeActive = false;
            state.activeHomeWindow = null;
        }

        state.motionState = newMotionState;
    }

    private static void handleRequestedExtendedTransition(
            LogSummary summary,
            AnalysisState state,
            long timestampUs,
            boolean requestedExtended) {
        if (state.lastRequestedToggleUs != Long.MIN_VALUE) {
            double deltaSec = (timestampUs - state.lastRequestedToggleUs) / 1_000_000.0;
            if (deltaSec <= 0.35) {
                summary.rapidRequestReversals.add(String.format(
                        Locale.US,
                        "t=%.3f requestedExtended=%s after %.3fs left=%.2f right=%.2f motion=%s",
                        seconds(timestampUs),
                        requestedExtended,
                        deltaSec,
                        state.leftPositionRot,
                        state.rightPositionRot,
                        state.motionState));
            }
        }
        state.lastRequestedToggleUs = timestampUs;

        if (state.pendingTransition != null && !state.pendingTransition.completed) {
            summary.supersededTransitions.add(state.pendingTransition.describe(timestampUs));
        }

        state.requestedExtended = requestedExtended;
        TransitionWindow window = new TransitionWindow(
                requestedExtended ? "EXTEND" : "RETRACT",
                timestampUs,
                requestedExtended ? EXTENDED_TARGET_ROT : RETRACTED_TARGET_ROT,
                state.mode,
                state.matchTimeSec);
        state.pendingTransition = window;
        summary.transitionWindows.add(window);
        summary.requestedStateEvents.add(String.format(
                Locale.US,
                "t=%.3f requestedExtended=%s left=%.2f right=%.2f goal=%s motion=%s",
                seconds(timestampUs),
                requestedExtended,
                state.leftPositionRot,
                state.rightPositionRot,
                state.goalState,
                state.motionState));
    }

    private static void emitStartedCommand(LogSummary summary, AnalysisState state, long timestampUs, int runId) {
        String name = emptyIfNull(state.pendingLastStartedName);
        String source = emptyIfNull(state.pendingLastStartedSource);
        String requirements = emptyIfNull(state.pendingLastStartedRequirements);
        boolean intakeRelevant = isIntakeRelevantCommand(name, requirements);

        CommandRun commandRun = new CommandRun(runId, name, source, requirements, timestampUs, intakeRelevant);
        state.commandRunsById.put(runId, commandRun);
        if (intakeRelevant) {
            summary.intakeCommandsStarted.add(commandRun);
        }
    }

    private static void emitEndedCommand(LogSummary summary, AnalysisState state, long timestampUs, int runId) {
        String name = emptyIfNull(state.pendingLastEndedName);
        String source = emptyIfNull(state.pendingLastEndedSource);
        boolean interrupted = state.pendingLastEndedInterrupted;
        double durationSec = state.pendingLastEndedDurationSec;

        CommandRun startedRun = state.commandRunsById.get(runId);
        boolean intakeRelevant = startedRun != null ? startedRun.intakeRelevant : isIntakeRelevantCommand(name, "");
        if (!intakeRelevant) {
            return;
        }

        String effectiveName = startedRun != null && !startedRun.name.isBlank() ? startedRun.name : name;
        String effectiveSource = startedRun != null && !startedRun.source.isBlank() ? startedRun.source : source;
        CommandEnd end = new CommandEnd(runId, effectiveName, effectiveSource, interrupted, durationSec, timestampUs);
        summary.intakeCommandsEnded.add(end);
        summary.commandCounts.merge(effectiveName, 1, Integer::sum);
        if (interrupted) {
            summary.interruptedCommandCounts.merge(effectiveName, 1, Integer::sum);
            if (isSuspiciousInterrupt(effectiveName)) {
                summary.suspiciousInterrupts.add(String.format(
                        Locale.US,
                        "t=%.3f %s interrupted duration=%.3fs source=%s mode=%s requested=%s motion=%s left=%.2f right=%.2f",
                        seconds(timestampUs),
                        effectiveName,
                        durationSec,
                        effectiveSource,
                        state.mode,
                        state.requestedExtended,
                        state.motionState,
                        state.leftPositionRot,
                        state.rightPositionRot));
            }
        }
    }

    private static void observeSnapshot(LogSummary summary, AnalysisState state, long timestampUs) {
        if (!Double.isFinite(state.leftPositionRot) || !Double.isFinite(state.rightPositionRot)) {
            return;
        }

        double symmetryError = Math.abs(state.leftPositionRot + state.rightPositionRot);
        summary.maxSymmetryErrorRot = Math.max(summary.maxSymmetryErrorRot, symmetryError);
        if (symmetryError > 0.45 && summary.symmetryOutliers.size() < 10) {
            summary.symmetryOutliers.add(String.format(
                    Locale.US,
                    "t=%.3f symmetryError=%.3f left=%.2f right=%.2f motion=%s requested=%s",
                    seconds(timestampUs),
                    symmetryError,
                    state.leftPositionRot,
                    state.rightPositionRot,
                    state.motionState,
                    state.requestedExtended));
        }

        double effectiveTargetRot = resolveEffectiveTargetRot(state);
        if (state.enabled
                && !state.smartRetractSessionActive
                && Double.isFinite(effectiveTargetRot)
                && !"HOMING".equals(state.motionState)) {
            boolean overlayActive = isOverlayActive(state);
            double leftError = Math.abs(state.leftPositionRot - effectiveTargetRot);
            double rightError = Math.abs(state.rightPositionRot + effectiveTargetRot);
            double maxSideError = Math.max(leftError, rightError);
            summary.maxTrackingErrorRot = Math.max(summary.maxTrackingErrorRot, maxSideError);
            summary.trackingErrorStats.add(maxSideError);

            if (overlayActive) {
                summary.overlayTrackingErrorStats.add(maxSideError);
            } else if ("EXTENDED".equals(state.motionState)) {
                summary.extendedTrackingErrorStats.add(maxSideError);
            } else if ("RETRACTED".equals(state.motionState)) {
                summary.retractedTrackingErrorStats.add(maxSideError);
            }

            if (!overlayActive
                    && ("EXTENDED".equals(state.motionState) || "RETRACTED".equals(state.motionState))
                    && maxSideError > 1.0
                    && summary.trackingOutliers.size() < 12) {
                summary.trackingOutliers.add(String.format(
                        Locale.US,
                        "t=%.3f target=%.2f error=%.3f left=%.2f right=%.2f requested=%s motion=%s goal=%s",
                        seconds(timestampUs),
                        effectiveTargetRot,
                        maxSideError,
                        state.leftPositionRot,
                        state.rightPositionRot,
                        state.requestedExtended,
                        state.motionState,
                        state.goalState));
            }
        }

        if (state.pendingTransition != null && !state.pendingTransition.completed
                && isAtTarget(state.leftPositionRot, state.rightPositionRot, state.pendingTransition.targetRot)) {
            state.pendingTransition.completed = true;
            state.pendingTransition.completedUs = timestampUs;
            double durationSec = (timestampUs - state.pendingTransition.startUs) / 1_000_000.0;
            summary.transitionDurationStats.add(durationSec);
            if ("EXTEND".equals(state.pendingTransition.label)) {
                summary.extendTransitionDurationStats.add(durationSec);
            } else {
                summary.retractTransitionDurationStats.add(durationSec);
            }
        }
    }

    private static void observeRoller(LogSummary summary, AnalysisState state, long timestampUs, double rollerVelocityRpm) {
        double absRpm = Math.abs(rollerVelocityRpm);
        boolean rollerCommandRunning = isRunning(state, "IntakeSpinRoller") || isRunning(state, "IntakeAgitate");
        boolean smartRetractRunning = state.smartRetractSessionActive || isRunning(state, "IntakeSmartRetractDuringShoot");

        if (rollerCommandRunning) {
            summary.activeRollerVelocityStats.add(absRpm);
        } else if (smartRetractRunning) {
            summary.smartRetractRollerVelocityStats.add(absRpm);
        } else {
            summary.passiveRollerVelocityStats.add(absRpm);
        }

        if (!state.enabled) {
            maybeCaptureDisabledActuation(summary, timestampUs, state, absRpm, "rollerRpm", rollerVelocityRpm);
        }
    }

    private static void maybeCaptureDisabledActuation(
            LogSummary summary,
            long timestampUs,
            AnalysisState state,
            double magnitude,
            String signal,
            double value) {
        summary.maxDisabledActuationMagnitude = Math.max(summary.maxDisabledActuationMagnitude, magnitude);
        if (magnitude > 0.75 && summary.disabledActuationSnapshots.size() < 12) {
            summary.disabledActuationSnapshots.add(String.format(
                    Locale.US,
                    "t=%.3f %s=%.2f mode=%s requested=%s motion=%s left=%.2f right=%.2f roller=%.0f",
                    seconds(timestampUs),
                    signal,
                    value,
                    state.mode,
                    state.requestedExtended,
                    state.motionState,
                    state.leftPositionRot,
                    state.rightPositionRot,
                    state.rollerVelocityRpm));
        }
    }

    private static double resolveEffectiveTargetRot(AnalysisState state) {
        if (state.driverAgitationActive && Double.isFinite(state.driverAgitationTargetRot)) {
            return state.driverAgitationTargetRot;
        }
        if (state.driverTriggerWiggleActive && Double.isFinite(state.driverTriggerWiggleTargetRot)) {
            return state.driverTriggerWiggleTargetRot;
        }
        if (state.smartRetractWiggleActive && Double.isFinite(state.smartRetractWiggleTargetRot)) {
            return state.smartRetractWiggleTargetRot;
        }
        if (state.smartRetractSessionActive && Double.isFinite(state.smartRetractCommandedTargetRot)) {
            return state.smartRetractCommandedTargetRot;
        }
        if ("EXTENDED".equals(state.goalState) || state.requestedExtended) {
            return EXTENDED_TARGET_ROT;
        }
        if ("RETRACTED".equals(state.goalState) || !state.requestedExtended) {
            return RETRACTED_TARGET_ROT;
        }
        return Double.NaN;
    }

    private static boolean isOverlayActive(AnalysisState state) {
        return state.positionOverlayActive
                || state.driverTriggerWiggleActive
                || state.driverAgitationActive
                || state.smartRetractWiggleActive;
    }

    private static boolean isAtTarget(double leftPositionRot, double rightPositionRot, double targetRot) {
        return Math.abs(leftPositionRot - targetRot) <= POSITION_TOLERANCE_ROT
                && Math.abs(rightPositionRot + targetRot) <= POSITION_TOLERANCE_ROT;
    }

    private static boolean isIntakeRelevantCommand(String name, String requirements) {
        if (name != null && name.contains("Intake")) {
            return true;
        }
        return requirements != null && requirements.contains("intake");
    }

    private static boolean isSuspiciousInterrupt(String name) {
        if (name == null || name.isBlank()) {
            return false;
        }
        return switch (name) {
            case "IntakeBackground",
                    "IntakeSpinRoller",
                    "IntakeAgitate",
                    "IntakeSmartRetractDuringShoot",
                    "DriverIntakeTriggerPress" -> false;
            default -> name.contains("IntakeHome")
                    || name.contains("IntakeExtend")
                    || name.contains("IntakeRetract")
                    || name.contains("StopAndRetract")
                    || name.contains("StopManipulators");
        };
    }

    private static boolean isRunning(AnalysisState state, String commandName) {
        return state.runningCommands.getOrDefault(commandName, false);
    }

    private static void updateGap(GapStats stats, long previousTimestampUs, long timestampUs) {
        if (previousTimestampUs == Long.MIN_VALUE) {
            return;
        }
        stats.maxGapSec = Math.max(stats.maxGapSec, (timestampUs - previousTimestampUs) / 1_000_000.0);
    }

    private static EntryIds resolveEntries(Map<Integer, EntryInfo> entries) {
        EntryIds ids = new EntryIds();
        ids.enabledEntry = findEntry(entries, "/RobotState/Enabled");
        ids.modeEntry = findEntry(entries, "/RobotState/Mode");
        ids.matchTimeEntry = findEntry(entries, "/RobotState/MatchTime");

        ids.motionStateEntry = findEntry(entries, "/Intake/MotionState");
        ids.goalStateEntry = findEntry(entries, "/Intake/GoalState");
        ids.requestedExtendedEntry = findEntry(entries, "/Intake/RequestedExtended");
        ids.extendedEntry = findEntry(entries, "/Intake/Extended");
        ids.positionOverlayActiveEntry = findEntry(entries, "/Intake/PositionOverlayActive");

        ids.leftPositionEntry = findEntry(entries, "/Intake/LeftPositionRad");
        ids.rightPositionEntry = findEntry(entries, "/Intake/RightPositionRad");
        ids.leftAppliedVoltsEntry = findEntry(entries, "/Intake/LeftAppliedVolts");
        ids.rightAppliedVoltsEntry = findEntry(entries, "/Intake/RightAppliedVolts");
        ids.leftCurrentEntry = findEntry(entries, "/Intake/LeftStatorCurrentAmps");
        ids.rightCurrentEntry = findEntry(entries, "/Intake/RightStatorCurrentAmps");
        ids.rollerVelocityEntry = findEntry(entries, "/Intake/RollerVelocityRpm");
        ids.rollerAppliedVoltsEntry = findEntry(entries, "/Intake/RollerAppliedVolts");
        ids.rollerCurrentEntry = findEntry(entries, "/Intake/RollerStatorCurrentAmps");

        ids.driverTriggerWiggleActiveEntry = findEntry(entries, "/Intake/DriverTriggerWiggleActive");
        ids.driverTriggerWiggleTargetEntry = findEntry(entries, "/Intake/DriverTriggerWiggleTargetRot");
        ids.driverAgitationActiveEntry = findEntry(entries, "/Intake/DriverAgitationActive");
        ids.driverAgitationTargetEntry = findEntry(entries, "/Intake/DriverAgitationTargetRot");
        ids.smartRetractSessionActiveEntry = findEntry(entries, "/Intake/SmartRetract/SessionActive");
        ids.smartRetractSessionModeEntry = findEntry(entries, "/Intake/SmartRetract/SessionMode");
        ids.smartRetractCommandedTargetEntry = findEntry(entries, "/Intake/SmartRetract/CommandedTargetRot");
        ids.smartRetractWiggleActiveEntry = findEntry(entries, "/Intake/SmartRetract/WiggleActive");
        ids.smartRetractWiggleTargetEntry = findEntry(entries, "/Intake/SmartRetract/WiggleTargetRot");

        ids.lastStartedNameEntry = findEntry(entries, "/Commands/lastStarted/name");
        ids.lastStartedSourceEntry = findEntry(entries, "/Commands/lastStarted/source");
        ids.lastStartedRequirementsEntry = findEntry(entries, "/Commands/lastStarted/requirements");
        ids.lastStartedRunIdEntry = findEntry(entries, "/Commands/lastStarted/runId");
        ids.lastEndedNameEntry = findEntry(entries, "/Commands/lastEnded/name");
        ids.lastEndedSourceEntry = findEntry(entries, "/Commands/lastEnded/source");
        ids.lastEndedInterruptedEntry = findEntry(entries, "/Commands/lastEnded/interrupted");
        ids.lastEndedDurationEntry = findEntry(entries, "/Commands/lastEnded/durationSec");
        ids.lastEndedRunIdEntry = findEntry(entries, "/Commands/lastEnded/runId");

        for (var entry : entries.entrySet()) {
            String name = entry.getValue().name;
            if (!name.contains("/Commands/byName/") || !name.endsWith("/running")) {
                continue;
            }
            String commandName = extractCommandNameFromRunningKey(name);
            ids.commandRunningEntries.put(entry.getKey(), commandName);
            if (commandName.contains("Intake")) {
                ids.runningCommandNames.add(commandName);
            }
        }
        Collections.sort(ids.runningCommandNames);
        return ids;
    }

    private static String extractCommandNameFromRunningKey(String fullEntryName) {
        int start = fullEntryName.indexOf("/Commands/byName/");
        int end = fullEntryName.lastIndexOf("/running");
        if (start < 0 || end < 0 || end <= start) {
            return fullEntryName;
        }
        return fullEntryName.substring(start + "/Commands/byName/".length(), end);
    }

    private static int findEntry(Map<Integer, EntryInfo> entries, String suffix) {
        for (var entry : entries.entrySet()) {
            if (entry.getValue().name.endsWith(suffix)) {
                return entry.getKey();
            }
        }
        return -1;
    }

    private static OptionalInt parseQualificationMatch(Path path) {
        Matcher matcher = QUAL_MATCH_PATTERN.matcher(path.getFileName().toString());
        if (!matcher.find()) {
            return OptionalInt.empty();
        }
        return OptionalInt.of(Integer.parseInt(matcher.group(1)));
    }

    private static double seconds(long timestampUs) {
        return timestampUs / 1_000_000.0;
    }

    private static String emptyIfNull(String value) {
        return value == null ? "" : value;
    }

    private static String formatAggregateReport(Path directory, List<LogSummary> summaries) {
        StringBuilder builder = new StringBuilder();
        builder.append("Intake audit across logs/mdbet after match 7").append(System.lineSeparator());
        builder.append("directory=").append(directory).append(System.lineSeparator());
        builder.append("logs=").append(summaries.size()).append(System.lineSeparator());

        Distribution extendDurations = new Distribution();
        Distribution retractDurations = new Distribution();
        Distribution allDurations = new Distribution();
        Distribution activeRollerRpm = new Distribution();
        Distribution passiveRollerRpm = new Distribution();
        Distribution smartRetractRollerRpm = new Distribution();
        Distribution extendedError = new Distribution();
        Distribution retractedError = new Distribution();
        Distribution overlayError = new Distribution();
        double worstSymmetryError = 0.0;
        double worstTrackingError = 0.0;
        double worstDisabledActuation = 0.0;
        int smartRetractSessions = 0;
        int homeWindows = 0;
        int suspiciousInterruptCount = 0;
        int incompleteTransitionCount = 0;
        int rapidReversalCount = 0;

        Set<String> commandNames = new HashSet<>();

        for (LogSummary summary : summaries) {
            extendDurations.addAll(summary.extendTransitionDurationStats.values);
            retractDurations.addAll(summary.retractTransitionDurationStats.values);
            allDurations.addAll(summary.transitionDurationStats.values);
            activeRollerRpm.addAll(summary.activeRollerVelocityStats.values);
            passiveRollerRpm.addAll(summary.passiveRollerVelocityStats.values);
            smartRetractRollerRpm.addAll(summary.smartRetractRollerVelocityStats.values);
            extendedError.addAll(summary.extendedTrackingErrorStats.values);
            retractedError.addAll(summary.retractedTrackingErrorStats.values);
            overlayError.addAll(summary.overlayTrackingErrorStats.values);
            worstSymmetryError = Math.max(worstSymmetryError, summary.maxSymmetryErrorRot);
            worstTrackingError = Math.max(worstTrackingError, summary.maxTrackingErrorRot);
            worstDisabledActuation = Math.max(worstDisabledActuation, summary.maxDisabledActuationMagnitude);
            smartRetractSessions += summary.smartRetractSessionCount;
            homeWindows += summary.homeWindows.size();
            suspiciousInterruptCount += summary.suspiciousInterrupts.size();
            incompleteTransitionCount += summary.incompleteTransitions.size();
            rapidReversalCount += summary.rapidRequestReversals.size();
            commandNames.addAll(summary.commandCounts.keySet());
        }

        builder.append("Aggregate").append(System.lineSeparator());
        builder.append(String.format(Locale.US, "  totalEnabledTime=%.1fs%n", summaries.stream().mapToDouble(s -> s.enabledTimeSec).sum()));
        builder.append(String.format(Locale.US, "  completedTransitions=%d %s%n",
                allDurations.values.size(), allDurations.describeSeconds()));
        builder.append(String.format(Locale.US, "  extendTransitions=%d %s%n",
                extendDurations.values.size(), extendDurations.describeSeconds()));
        builder.append(String.format(Locale.US, "  retractTransitions=%d %s%n",
                retractDurations.values.size(), retractDurations.describeSeconds()));
        builder.append(String.format(Locale.US, "  extendedTrackingError=%s%n", extendedError.describeRot()));
        builder.append(String.format(Locale.US, "  retractedTrackingError=%s%n", retractedError.describeRot()));
        builder.append(String.format(Locale.US, "  overlayTrackingError=%s%n", overlayError.describeRot()));
        builder.append(String.format(Locale.US, "  activeRollerRpm=%s%n", activeRollerRpm.describeRpm()));
        builder.append(String.format(Locale.US, "  passiveRollerRpm=%s%n", passiveRollerRpm.describeRpm()));
        builder.append(String.format(Locale.US, "  smartRetractRollerRpm=%s%n", smartRetractRollerRpm.describeRpm()));
        builder.append(String.format(Locale.US, "  smartRetractSessions=%d%n", smartRetractSessions));
        builder.append(String.format(Locale.US, "  homeWindows=%d%n", homeWindows));
        builder.append(String.format(Locale.US, "  worstSymmetryError=%.3f rot%n", worstSymmetryError));
        builder.append(String.format(Locale.US, "  worstTrackingError=%.3f rot%n", worstTrackingError));
        builder.append(String.format(Locale.US, "  worstDisabledActuation=%.2f%n", worstDisabledActuation));
        builder.append(String.format(Locale.US, "  suspiciousInterrupts=%d%n", suspiciousInterruptCount));
        builder.append(String.format(Locale.US, "  incompleteTransitions=%d%n", incompleteTransitionCount));
        builder.append(String.format(Locale.US, "  rapidRequestReversals=%d%n", rapidReversalCount));
        builder.append("  intakeCommandsSeen=").append(new ArrayList<>(commandNames)).append(System.lineSeparator());
        builder.append(System.lineSeparator());

        for (LogSummary summary : summaries) {
            builder.append(summary.format()).append(System.lineSeparator());
        }
        return builder.toString();
    }

    private record EntryInfo(String name, String type) {}

    private static final class EntryIds {
        private int enabledEntry = -1;
        private int modeEntry = -1;
        private int matchTimeEntry = -1;
        private int motionStateEntry = -1;
        private int goalStateEntry = -1;
        private int requestedExtendedEntry = -1;
        private int extendedEntry = -1;
        private int positionOverlayActiveEntry = -1;
        private int leftPositionEntry = -1;
        private int rightPositionEntry = -1;
        private int leftAppliedVoltsEntry = -1;
        private int rightAppliedVoltsEntry = -1;
        private int leftCurrentEntry = -1;
        private int rightCurrentEntry = -1;
        private int rollerVelocityEntry = -1;
        private int rollerAppliedVoltsEntry = -1;
        private int rollerCurrentEntry = -1;
        private int driverTriggerWiggleActiveEntry = -1;
        private int driverTriggerWiggleTargetEntry = -1;
        private int driverAgitationActiveEntry = -1;
        private int driverAgitationTargetEntry = -1;
        private int smartRetractSessionActiveEntry = -1;
        private int smartRetractSessionModeEntry = -1;
        private int smartRetractCommandedTargetEntry = -1;
        private int smartRetractWiggleActiveEntry = -1;
        private int smartRetractWiggleTargetEntry = -1;
        private int lastStartedNameEntry = -1;
        private int lastStartedSourceEntry = -1;
        private int lastStartedRequirementsEntry = -1;
        private int lastStartedRunIdEntry = -1;
        private int lastEndedNameEntry = -1;
        private int lastEndedSourceEntry = -1;
        private int lastEndedInterruptedEntry = -1;
        private int lastEndedDurationEntry = -1;
        private int lastEndedRunIdEntry = -1;
        private final Map<Integer, String> commandRunningEntries = new HashMap<>();
        private final List<String> runningCommandNames = new ArrayList<>();
    }

    private static final class AnalysisState {
        private boolean enabled = false;
        private String mode = "";
        private double matchTimeSec = Double.NaN;
        private String motionState = "";
        private String goalState = "";
        private boolean requestedExtended = false;
        private boolean extended = false;
        private boolean positionOverlayActive = false;
        private boolean driverTriggerWiggleActive = false;
        private boolean driverAgitationActive = false;
        private boolean smartRetractSessionActive = false;
        private boolean smartRetractWiggleActive = false;
        private String smartRetractSessionMode = "";

        private double leftPositionRot = Double.NaN;
        private double rightPositionRot = Double.NaN;
        private double leftAppliedVolts = Double.NaN;
        private double rightAppliedVolts = Double.NaN;
        private double leftCurrentAmps = Double.NaN;
        private double rightCurrentAmps = Double.NaN;
        private double rollerVelocityRpm = Double.NaN;
        private double rollerAppliedVolts = Double.NaN;
        private double rollerCurrentAmps = Double.NaN;
        private double driverTriggerWiggleTargetRot = Double.NaN;
        private double driverAgitationTargetRot = Double.NaN;
        private double smartRetractCommandedTargetRot = Double.NaN;
        private double smartRetractWiggleTargetRot = Double.NaN;

        private long enabledSinceUs = Long.MIN_VALUE;
        private long lastRequestedToggleUs = Long.MIN_VALUE;
        private long lastLeftPositionTsUs = Long.MIN_VALUE;
        private long lastRightPositionTsUs = Long.MIN_VALUE;
        private long lastRollerVelocityTsUs = Long.MIN_VALUE;
        private long previousRecordTsUs = Long.MIN_VALUE;

        private TransitionWindow pendingTransition;
        private boolean homeActive = false;
        private HomeWindow activeHomeWindow;

        private String pendingLastStartedName = "";
        private String pendingLastStartedSource = "";
        private String pendingLastStartedRequirements = "";
        private String pendingLastEndedName = "";
        private String pendingLastEndedSource = "";
        private boolean pendingLastEndedInterrupted = false;
        private double pendingLastEndedDurationSec = 0.0;

        private final Map<Integer, CommandRun> commandRunsById = new HashMap<>();
        private final Map<String, Boolean> runningCommands = new HashMap<>();
    }

    private static final class LogSummary {
        private final Path wpilog;
        private final int qualificationMatch;
        private boolean truncated = false;
        private long firstTimestampUs = Long.MAX_VALUE;
        private long lastTimestampUs = Long.MIN_VALUE;
        private double enabledTimeSec = 0.0;
        private int smartRetractSessionCount = 0;
        private double maxSymmetryErrorRot = 0.0;
        private double maxTrackingErrorRot = 0.0;
        private double maxDisabledActuationMagnitude = 0.0;

        private final Distribution leftPositionStats = new Distribution();
        private final Distribution rightPositionStats = new Distribution();
        private final Distribution leftVoltageStats = new Distribution();
        private final Distribution rightVoltageStats = new Distribution();
        private final Distribution leftCurrentStats = new Distribution();
        private final Distribution rightCurrentStats = new Distribution();
        private final Distribution rollerVelocityStats = new Distribution();
        private final Distribution rollerVoltageStats = new Distribution();
        private final Distribution rollerCurrentStats = new Distribution();
        private final Distribution activeRollerVelocityStats = new Distribution();
        private final Distribution passiveRollerVelocityStats = new Distribution();
        private final Distribution smartRetractRollerVelocityStats = new Distribution();
        private final Distribution transitionDurationStats = new Distribution();
        private final Distribution extendTransitionDurationStats = new Distribution();
        private final Distribution retractTransitionDurationStats = new Distribution();
        private final Distribution trackingErrorStats = new Distribution();
        private final Distribution extendedTrackingErrorStats = new Distribution();
        private final Distribution retractedTrackingErrorStats = new Distribution();
        private final Distribution overlayTrackingErrorStats = new Distribution();
        private final Distribution smartRetractCommandedTargetStats = new Distribution();
        private final GapStats leftPositionGapStats = new GapStats();
        private final GapStats rightPositionGapStats = new GapStats();
        private final GapStats rollerVelocityGapStats = new GapStats();

        private final List<String> modeEvents = new ArrayList<>();
        private final List<String> motionStateEvents = new ArrayList<>();
        private final List<String> requestedStateEvents = new ArrayList<>();
        private final List<String> extendedStateEvents = new ArrayList<>();
        private final List<String> disabledActuationSnapshots = new ArrayList<>();
        private final List<String> symmetryOutliers = new ArrayList<>();
        private final List<String> trackingOutliers = new ArrayList<>();
        private final List<String> suspiciousInterrupts = new ArrayList<>();
        private final List<String> incompleteTransitions = new ArrayList<>();
        private final List<String> supersededTransitions = new ArrayList<>();
        private final List<String> rapidRequestReversals = new ArrayList<>();
        private final List<String> discoveredIntakeCommands = new ArrayList<>();
        private final List<TransitionWindow> transitionWindows = new ArrayList<>();
        private final List<HomeWindow> homeWindows = new ArrayList<>();
        private final List<CommandRun> intakeCommandsStarted = new ArrayList<>();
        private final List<CommandEnd> intakeCommandsEnded = new ArrayList<>();
        private final Map<String, Integer> commandCounts = new LinkedHashMap<>();
        private final Map<String, Integer> interruptedCommandCounts = new LinkedHashMap<>();

        private LogSummary(Path wpilog, int qualificationMatch) {
            this.wpilog = wpilog;
            this.qualificationMatch = qualificationMatch;
        }

        private String format() {
            StringBuilder builder = new StringBuilder();
            builder.append(String.format(Locale.US, "Match q%d - %s%n", qualificationMatch, wpilog.getFileName()));
            builder.append(String.format(Locale.US, "  truncated=%s enabledTime=%.1fs logSpan=%.1fs%n",
                    truncated,
                    enabledTimeSec,
                    firstTimestampUs == Long.MAX_VALUE || lastTimestampUs == Long.MIN_VALUE
                            ? 0.0
                            : (lastTimestampUs - firstTimestampUs) / 1_000_000.0));
            builder.append(String.format(Locale.US, "  transitions total=%d complete=%d extend=%d retract=%d incomplete=%d rapidReversals=%d%n",
                    transitionWindows.size(),
                    transitionDurationStats.values.size(),
                    extendTransitionDurationStats.values.size(),
                    retractTransitionDurationStats.values.size(),
                    incompleteTransitions.size(),
                    rapidRequestReversals.size()));
            builder.append(String.format(Locale.US, "  transitionDurations=%s%n", transitionDurationStats.describeSeconds()));
            builder.append(String.format(Locale.US, "  extendDurations=%s%n", extendTransitionDurationStats.describeSeconds()));
            builder.append(String.format(Locale.US, "  retractDurations=%s%n", retractTransitionDurationStats.describeSeconds()));
            builder.append(String.format(Locale.US, "  symmetry worst=%.3f rot tracking worst=%.3f rot%n",
                    maxSymmetryErrorRot,
                    maxTrackingErrorRot));
            builder.append(String.format(Locale.US, "  extendedError=%s%n", extendedTrackingErrorStats.describeRot()));
            builder.append(String.format(Locale.US, "  retractedError=%s%n", retractedTrackingErrorStats.describeRot()));
            builder.append(String.format(Locale.US, "  overlayError=%s%n", overlayTrackingErrorStats.describeRot()));
            builder.append(String.format(Locale.US, "  disabledActuation worst=%.2f snapshots=%d%n",
                    maxDisabledActuationMagnitude,
                    disabledActuationSnapshots.size()));
            builder.append(String.format(Locale.US, "  positionGaps left=%.3fs right=%.3fs roller=%.3fs%n",
                    leftPositionGapStats.maxGapSec,
                    rightPositionGapStats.maxGapSec,
                    rollerVelocityGapStats.maxGapSec));
            builder.append(String.format(Locale.US, "  leftPosition=%s%n", leftPositionStats.describeRot()));
            builder.append(String.format(Locale.US, "  rightPosition=%s%n", rightPositionStats.describeRot()));
            builder.append(String.format(Locale.US, "  leftCurrent=%s%n", leftCurrentStats.describeAmps()));
            builder.append(String.format(Locale.US, "  rightCurrent=%s%n", rightCurrentStats.describeAmps()));
            builder.append(String.format(Locale.US, "  rollerVelocity=%s%n", rollerVelocityStats.describeRpm()));
            builder.append(String.format(Locale.US, "  activeRollerVelocity=%s%n", activeRollerVelocityStats.describeRpm()));
            builder.append(String.format(Locale.US, "  passiveRollerVelocity=%s%n", passiveRollerVelocityStats.describeRpm()));
            builder.append(String.format(Locale.US, "  smartRetractRollerVelocity=%s%n", smartRetractRollerVelocityStats.describeRpm()));
            builder.append(String.format(Locale.US, "  smartRetract sessions=%d targets=%s%n",
                    smartRetractSessionCount,
                    smartRetractCommandedTargetStats.describeRot()));
            builder.append(String.format(Locale.US, "  homeWindows=%d%n", homeWindows.size()));
            for (HomeWindow homeWindow : homeWindows) {
                builder.append(String.format(Locale.US,
                        "    home duration=%.3fs maxLeftI=%.1fA maxRightI=%.1fA%n",
                        homeWindow.durationSec(),
                        homeWindow.maxAbsLeftCurrentAmps,
                        homeWindow.maxAbsRightCurrentAmps));
            }
            builder.append("  intakeCommandsSeen=").append(discoveredIntakeCommands).append(System.lineSeparator());
            builder.append("  intakeCommandCounts=").append(commandCounts).append(System.lineSeparator());
            if (!interruptedCommandCounts.isEmpty()) {
                builder.append("  interruptedCommandCounts=").append(interruptedCommandCounts).append(System.lineSeparator());
            }
            appendSection(builder, "  suspiciousInterrupts", suspiciousInterrupts, 8);
            appendSection(builder, "  incompleteTransitions", incompleteTransitions, 8);
            appendSection(builder, "  supersededTransitions", supersededTransitions, 8);
            appendSection(builder, "  rapidRequestReversals", rapidRequestReversals, 8);
            appendSection(builder, "  disabledActuationSnapshots", disabledActuationSnapshots, 6);
            appendSection(builder, "  symmetryOutliers", symmetryOutliers, 6);
            appendSection(builder, "  trackingOutliers", trackingOutliers, 6);
            appendTransitionSection(builder, transitionWindows, 8);
            return builder.toString();
        }

        private static void appendSection(StringBuilder builder, String label, List<String> values, int limit) {
            if (values.isEmpty()) {
                return;
            }
            builder.append(label).append(System.lineSeparator());
            for (int i = 0; i < Math.min(limit, values.size()); i++) {
                builder.append("    ").append(values.get(i)).append(System.lineSeparator());
            }
            if (values.size() > limit) {
                builder.append("    ... ").append(values.size() - limit).append(" more").append(System.lineSeparator());
            }
        }

        private static void appendTransitionSection(StringBuilder builder, List<TransitionWindow> windows, int limit) {
            if (windows.isEmpty()) {
                return;
            }
            builder.append("  transitionsDetail").append(System.lineSeparator());
            for (int i = 0; i < Math.min(limit, windows.size()); i++) {
                builder.append("    ").append(windows.get(i).describe(windows.get(i).completed ? windows.get(i).completedUs : windows.get(i).startUs)).append(System.lineSeparator());
            }
            if (windows.size() > limit) {
                builder.append("    ... ").append(windows.size() - limit).append(" more").append(System.lineSeparator());
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

        private void addAll(List<Double> moreValues) {
            for (double value : moreValues) {
                add(value);
            }
        }

        private String describeSeconds() {
            return describe("s");
        }

        private String describeRot() {
            return describe("rot");
        }

        private String describeRpm() {
            return describe("rpm");
        }

        private String describeAmps() {
            return describe("A");
        }

        private String describe(String unit) {
            if (values.isEmpty()) {
                return "[]";
            }
            List<Double> sorted = new ArrayList<>(values);
            Collections.sort(sorted);
            double mean = sorted.stream().mapToDouble(Double::doubleValue).average().orElse(Double.NaN);
            return String.format(
                    Locale.US,
                    "count=%d min=%.3f%s p50=%.3f%s p90=%.3f%s max=%.3f%s mean=%.3f%s",
                    sorted.size(),
                    sorted.get(0),
                    unit,
                    percentile(sorted, 0.50),
                    unit,
                    percentile(sorted, 0.90),
                    unit,
                    sorted.get(sorted.size() - 1),
                    unit,
                    mean,
                    unit);
        }

        private static double percentile(List<Double> sorted, double fraction) {
            if (sorted.isEmpty()) {
                return Double.NaN;
            }
            if (sorted.size() == 1) {
                return sorted.get(0);
            }
            double index = fraction * (sorted.size() - 1);
            int lower = (int) Math.floor(index);
            int upper = (int) Math.ceil(index);
            if (lower == upper) {
                return sorted.get(lower);
            }
            double weight = index - lower;
            return sorted.get(lower) + weight * (sorted.get(upper) - sorted.get(lower));
        }
    }

    private static final class GapStats {
        private double maxGapSec = 0.0;
    }

    private static final class TransitionWindow {
        private final String label;
        private final long startUs;
        private final double targetRot;
        private final String mode;
        private final double matchTimeSec;
        private boolean completed = false;
        private long completedUs = Long.MIN_VALUE;

        private TransitionWindow(String label, long startUs, double targetRot, String mode, double matchTimeSec) {
            this.label = label;
            this.startUs = startUs;
            this.targetRot = targetRot;
            this.mode = mode;
            this.matchTimeSec = matchTimeSec;
        }

        private String describe(long endUs) {
            double durationSec = endUs == Long.MIN_VALUE ? Double.NaN : (endUs - startUs) / 1_000_000.0;
            return String.format(
                    Locale.US,
                    "%s target=%.2f mode=%s matchTime=%.1f start=%.3f duration=%.3fs completed=%s",
                    label,
                    targetRot,
                    mode,
                    matchTimeSec,
                    seconds(startUs),
                    durationSec,
                    completed);
        }
    }

    private static final class HomeWindow {
        private final long startUs;
        private long endUs = Long.MIN_VALUE;
        private double maxAbsLeftCurrentAmps = 0.0;
        private double maxAbsRightCurrentAmps = 0.0;

        private HomeWindow(long startUs) {
            this.startUs = startUs;
        }

        private double durationSec() {
            if (endUs == Long.MIN_VALUE) {
                return Double.NaN;
            }
            return (endUs - startUs) / 1_000_000.0;
        }
    }

    private record CommandRun(
            int runId,
            String name,
            String source,
            String requirements,
            long startUs,
            boolean intakeRelevant) {}

    private record CommandEnd(
            int runId,
            String name,
            String source,
            boolean interrupted,
            double durationSec,
            long timestampUs) {}
}
