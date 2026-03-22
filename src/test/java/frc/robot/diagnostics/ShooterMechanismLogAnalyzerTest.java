package frc.robot.diagnostics;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;

class ShooterMechanismLogAnalyzerTest {
    private static final Pattern QUALS_MATCH_PATTERN = Pattern.compile("_q(\\d+)\\.wpilog$");
    private static final double LOOP_EPSILON_SEC = 0.005;
    private static final double HIGH_TARGET_THRESHOLD_RPM = 2500.0;
    private static final double BACKGROUND_TARGET_THRESHOLD_RPM = 1500.0;
    private static final double TARGET_PRESENT_THRESHOLD_RPM = 500.0;
    private static final double KICKER_ACTIVE_VOLTS = 1.0;
    private static final double KICKER_ACTIVE_TORQUE_AMPS = 1.0;
    private static final double ZERO_TARGET_STREAK_THRESHOLD_SEC = 0.06;
    private static final double SPEED_COLLAPSE_STREAK_THRESHOLD_SEC = 0.10;
    private static final double COHERENCE_STREAK_THRESHOLD_SEC = 0.10;
    private static final int MAX_NOTABLE_EVENTS = 24;

    @Test
    void analyzeMdbetShooterMechanismAfterQual7() throws IOException {
        Path logDir = Path.of("logs/mdbet").toAbsolutePath();
        assertTrue(Files.isDirectory(logDir), "Missing log directory: " + logDir);

        List<Path> logs;
        try (Stream<Path> stream = Files.list(logDir)) {
            logs = stream.filter(path -> Files.isRegularFile(path) && includeLog(path))
                    .sorted(Comparator.comparing(Path::getFileName))
                    .collect(Collectors.toList());
        }

        assertFalse(logs.isEmpty(), "No logs found in " + logDir + " after qual 7.");

        List<MatchAnalysis> analyses = new ArrayList<>();
        for (Path log : logs) {
            analyses.add(analyze(log));
        }

        String report = formatAggregateReport(logDir, analyses);
        Path outDir = Path.of("build/diagnostics/shooter-mechanism");
        Files.createDirectories(outDir);
        Files.writeString(outDir.resolve("mdbet-after-q7-summary.txt"), report);
        Files.writeString(outDir.resolve("mdbet-after-q7-high-target-windows.csv"), buildHighTargetWindowCsv(analyses));
        Files.writeString(outDir.resolve("mdbet-after-q7-command-events.csv"), buildCommandEventCsv(analyses));

        System.out.println(report);
    }

    private static boolean includeLog(Path path) {
        Matcher matcher = QUALS_MATCH_PATTERN.matcher(path.getFileName().toString());
        if (!matcher.find()) {
            return false;
        }
        return Integer.parseInt(matcher.group(1)) > 7;
    }

    private static MatchAnalysis analyze(Path wpilog) throws IOException {
        DataLogReader reader = new DataLogReader(wpilog.toString());
        if (!reader.isValid()) {
            throw new IllegalStateException("Invalid WPILOG file: " + wpilog);
        }

        EntryIds entryIds = new EntryIds();
        Map<Integer, EntryInfo> entries = new HashMap<>();
        StringBuilder parseWarnings = new StringBuilder();

        Iterator<DataLogRecord> startIterator = reader.iterator();
        while (true) {
            DataLogRecord record;
            try {
                if (!startIterator.hasNext()) {
                    break;
                }
                record = startIterator.next();
            } catch (RuntimeException exception) {
                appendWarning(parseWarnings, "start scan terminated early: " + exception.getMessage());
                break;
            }
            if (!record.isStart()) {
                continue;
            }
            var start = record.getStartData();
            entries.put(start.entry, new EntryInfo(start.name, start.type));
            entryIds.capture(start.entry, start.name, start.type);
        }

        TimelineState state = new TimelineState(entryIds.commandRunningEntries);
        MatchStats stats = new MatchStats(matchLabel(wpilog), entryIds, entries, wpilog);

        DataLogReader dataReader = new DataLogReader(wpilog.toString());
        Iterator<DataLogRecord> iterator = dataReader.iterator();
        long currentTimestampUs = Long.MIN_VALUE;
        while (true) {
            final DataLogRecord record;
            try {
                if (!iterator.hasNext()) {
                    break;
                }
                record = iterator.next();
            } catch (RuntimeException exception) {
                appendWarning(parseWarnings, "data scan terminated early: " + exception.getMessage());
                break;
            }

            if (record.isStart() || record.isControl()) {
                continue;
            }

            long timestampUs = record.getTimestamp();
            if (currentTimestampUs != Long.MIN_VALUE && timestampUs + 1_000_000L < currentTimestampUs) {
                appendWarning(parseWarnings,
                        String.format(Locale.US,
                                "timestamp regressed from %.3fs to %.3fs; resetting interval state",
                                currentTimestampUs / 1_000_000.0,
                                timestampUs / 1_000_000.0));
                stats.finish(currentTimestampUs, state);
                currentTimestampUs = timestampUs;
                continue;
            }
            if (currentTimestampUs == Long.MIN_VALUE) {
                currentTimestampUs = timestampUs;
            } else if (timestampUs != currentTimestampUs) {
                stats.flushSample(currentTimestampUs, timestampUs, state);
                currentTimestampUs = timestampUs;
            }

            int entry = record.getEntry();
            if (entry == entryIds.modeEntry) {
                state.mode = readStringLenient(record);
                continue;
            }
            if (entry == entryIds.enabledEntry) {
                state.enabled = readBooleanLenient(record, entryName(entries, entry));
                continue;
            }
            if (entry == entryIds.matchTimeEntry) {
                state.matchTimeSec = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.batteryVoltageEntry) {
                state.batteryVoltage = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.pdpTotalCurrentEntry) {
                state.totalCurrentAmps = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.brownoutEntry) {
                state.brownedOut = readBooleanLenient(record, entryName(entries, entry));
                continue;
            }

            if (entry == entryIds.targetLeftRpmEntry) {
                state.targetLeftRpm = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.targetRightRpmEntry) {
                state.targetRightRpm = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.targetHoodDegEntry) {
                state.targetHoodDeg = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.kickerTorqueEntry) {
                state.kickerTorqueCommandAmps = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.kickerVoltageEntry) {
                state.kickerVoltageCommand = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.readinessLeftErrorEntry) {
                state.leftVelocityErrorRpm = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.readinessRightErrorEntry) {
                state.rightVelocityErrorRpm = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.readinessHoodErrorEntry) {
                state.hoodAngleErrorDeg = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.leftAtSetpointEntry) {
                state.leftVelocityAtSetpoint = readBooleanLenient(record, entryName(entries, entry));
                continue;
            }
            if (entry == entryIds.rightAtSetpointEntry) {
                state.rightVelocityAtSetpoint = readBooleanLenient(record, entryName(entries, entry));
                continue;
            }
            if (entry == entryIds.hoodAtSetpointEntry) {
                state.hoodAtSetpoint = readBooleanLenient(record, entryName(entries, entry));
                continue;
            }
            if (entry == entryIds.shooterAtSetpointEntry) {
                state.shooterAtSetpoint = readBooleanLenient(record, entryName(entries, entry));
                continue;
            }
            if (entry == entryIds.readyToFireEntry) {
                state.readyToFire = readBooleanLenient(record, entryName(entries, entry));
                continue;
            }
            if (entry == entryIds.readinessModeEntry) {
                state.readinessMode = readStringLenient(record);
                continue;
            }
            if (entry == entryIds.readinessToleranceEntry) {
                state.readinessToleranceRpm = readDoubleLenient(record);
                continue;
            }

            if (entry == entryIds.leftVelocityEntry) {
                state.leftVelocityRpm = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.rightVelocityEntry) {
                state.rightVelocityRpm = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.leftAppliedVoltsEntry) {
                state.leftAppliedVolts = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.rightAppliedVoltsEntry) {
                state.rightAppliedVolts = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.leftSupplyCurrentEntry) {
                state.leftSupplyCurrentAmps = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.rightSupplyCurrentEntry) {
                state.rightSupplyCurrentAmps = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.leftTempEntry) {
                state.leftTempC = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.rightTempEntry) {
                state.rightTempC = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.hoodPositionEntry) {
                state.hoodPositionDeg = Math.toDegrees(readDoubleLenient(record));
                continue;
            }
            if (entry == entryIds.hoodVelocityEntry) {
                state.hoodVelocityRpm = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.hoodAppliedVoltsEntry) {
                state.hoodAppliedVolts = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.hoodSupplyCurrentEntry) {
                state.hoodSupplyCurrentAmps = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.hoodStatorCurrentEntry) {
                state.hoodStatorCurrentAmps = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.hoodTempEntry) {
                state.hoodTempC = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.kickerVelocityEntry) {
                state.kickerVelocityRpm = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.kickerAppliedVoltsEntry) {
                state.kickerAppliedVolts = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.kickerSupplyCurrentEntry) {
                state.kickerSupplyCurrentAmps = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.kickerTempEntry) {
                state.kickerTempC = readDoubleLenient(record);
                continue;
            }

            if (entry == entryIds.shootingShooterAtSetpointEntry) {
                state.shootingShooterAtSetpoint = readBooleanLenient(record, entryName(entries, entry));
                continue;
            }
            if (entry == entryIds.aimReadyEntry) {
                state.aimReady = readBooleanLenient(record, entryName(entries, entry));
                continue;
            }
            if (entry == entryIds.gateOpenEntry) {
                state.gateOpen = readBooleanLenient(record, entryName(entries, entry));
                continue;
            }
            if (entry == entryIds.shootingStateEntry) {
                state.shootingState = readStringLenient(record);
                continue;
            }
            if (entry == entryIds.blockReasonEntry) {
                state.blockReason = readStringLenient(record);
                continue;
            }
            if (entry == entryIds.automaticFeedEnabledEntry) {
                state.automaticFeedEnabled = readBooleanLenient(record, entryName(entries, entry));
                continue;
            }
            if (entry == entryIds.manualFeedOverrideEntry) {
                state.manualFeedOverride = readBooleanLenient(record, entryName(entries, entry));
                continue;
            }

            if (entry == entryIds.lastStartedNameEntry) {
                state.lastStartedName = readStringLenient(record);
                continue;
            }
            if (entry == entryIds.lastStartedSourceEntry) {
                state.lastStartedSource = readStringLenient(record);
                continue;
            }
            if (entry == entryIds.lastStartedRequirementsEntry) {
                state.lastStartedRequirements = readStringLenient(record);
                continue;
            }
            if (entry == entryIds.lastStartedRunIdEntry) {
                long runId = readLongLenient(record);
                stats.recordCommandEvent(new CommandEvent(
                        timestampUs,
                        "START",
                        state.lastStartedName,
                        state.lastStartedSource,
                        state.lastStartedRequirements,
                        runId,
                        false,
                        Double.NaN,
                        state.snapshot()));
                continue;
            }
            if (entry == entryIds.lastEndedNameEntry) {
                state.lastEndedName = readStringLenient(record);
                continue;
            }
            if (entry == entryIds.lastEndedSourceEntry) {
                state.lastEndedSource = readStringLenient(record);
                continue;
            }
            if (entry == entryIds.lastEndedInterruptedEntry) {
                state.lastEndedInterrupted = readBooleanLenient(record, entryName(entries, entry));
                continue;
            }
            if (entry == entryIds.lastEndedDurationEntry) {
                state.lastEndedDurationSec = readDoubleLenient(record);
                continue;
            }
            if (entry == entryIds.lastEndedRunIdEntry) {
                long runId = readLongLenient(record);
                stats.recordCommandEvent(new CommandEvent(
                        timestampUs,
                        state.lastEndedInterrupted ? "INTERRUPT" : "FINISH",
                        state.lastEndedName,
                        state.lastEndedSource,
                        "",
                        runId,
                        state.lastEndedInterrupted,
                        state.lastEndedDurationSec,
                        state.snapshot()));
                continue;
            }

            String commandName = entryIds.commandRunningEntries.get(entry);
            if (commandName != null) {
                state.commandRunning.put(commandName, readBooleanLenient(record, entryName(entries, entry)));
            }
        }

        if (currentTimestampUs != Long.MIN_VALUE) {
            stats.finish(currentTimestampUs, state);
        }

        return stats.toAnalysis(parseWarnings.toString());
    }

    private static String matchLabel(Path wpilog) {
        String name = wpilog.getFileName().toString();
        Matcher matcher = QUALS_MATCH_PATTERN.matcher(name);
        if (matcher.find()) {
            return "q" + matcher.group(1);
        }
        return name;
    }

    private static void appendWarning(StringBuilder builder, String warning) {
        if (builder.length() > 0) {
            builder.append("; ");
        }
        builder.append(warning);
    }

    private static String entryName(Map<Integer, EntryInfo> entries, int entryId) {
        EntryInfo info = entries.get(entryId);
        return info == null ? "entry=" + entryId : info.name;
    }

    private static String readStringLenient(DataLogRecord record) {
        try {
            return record.getString();
        } catch (Exception ignored) {
        }
        try {
            return String.valueOf(record.getDouble());
        } catch (Exception ignored) {
        }
        try {
            return String.valueOf(record.getInteger());
        } catch (Exception ignored) {
        }
        return "";
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
        throw new IllegalStateException("Unable to read numeric record for entry " + record.getEntry());
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
        throw new IllegalStateException("Unable to read integer record for entry " + record.getEntry());
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
        throw new IllegalStateException("Unable to read boolean for " + name);
    }

    private static String formatAggregateReport(Path logDir, List<MatchAnalysis> analyses) {
        StringBuilder out = new StringBuilder();
        out.append("Shooter mechanism MDBET analysis after qual 7\n");
        out.append("logDir=").append(logDir).append('\n');
        out.append("matches=").append(analyses.stream().map(MatchAnalysis::label).collect(Collectors.joining(", "))).append('\n');

        SummaryAccumulator spinup = new SummaryAccumulator();
        SummaryAccumulator steadyAvgError = new SummaryAccumulator();
        SummaryAccumulator steadyHoodError = new SummaryAccumulator();
        SummaryAccumulator leftRightDelta = new SummaryAccumulator();
        SummaryAccumulator battery = new SummaryAccumulator();
        double totalEnabled = 0.0;
        double totalHighTarget = 0.0;
        double totalGateOpen = 0.0;
        double totalKickerActive = 0.0;
        double totalMismatch = 0.0;
        int totalBrownouts = 0;
        int totalDropouts = 0;
        int totalCollapses = 0;
        int totalUnexpectedInterrupts = 0;

        for (MatchAnalysis analysis : analyses) {
            totalEnabled += analysis.enabledSec();
            totalHighTarget += analysis.highTargetSec();
            totalGateOpen += analysis.gateOpenSec();
            totalKickerActive += analysis.kickerActiveSec();
            totalMismatch += analysis.gateKickerMismatchSec();
            totalBrownouts += analysis.brownoutCount();
            totalDropouts += analysis.zeroTargetDropouts().size();
            totalCollapses += analysis.speedCollapseEvents().size();
            totalUnexpectedInterrupts += analysis.unexpectedInterrupts().size();
            spinup.addAll(analysis.highTargetWindows().stream()
                    .map(HighTargetWindow::spinupSec)
                    .filter(Double::isFinite)
                    .collect(Collectors.toList()));
            steadyAvgError.addAll(analysis.steadyAvgErrorRpmSamples());
            steadyHoodError.addAll(analysis.steadyHoodErrorDegSamples());
            leftRightDelta.addAll(analysis.steadyLeftRightVelocityDeltaSamples());
            battery.addAll(analysis.shooterActiveBatterySamples());
        }

        out.append("aggregate\n");
        out.append(String.format(Locale.US,
                "  enabledSec=%.1f highTargetSec=%.1f gateOpenSec=%.1f kickerActiveSec=%.1f mismatchSec=%.3f brownouts=%d\n",
                totalEnabled,
                totalHighTarget,
                totalGateOpen,
                totalKickerActive,
                totalMismatch,
                totalBrownouts));
        out.append(String.format(Locale.US,
                "  highTargetWindows=%d spinup=%s steadyAvgErrorRpm=%s steadyHoodErrorDeg=%s leftRightDeltaRpm=%s shooterActiveBattery=%s\n",
                analyses.stream().mapToInt(a -> a.highTargetWindows().size()).sum(),
                spinup.describe(),
                steadyAvgError.describe(),
                steadyHoodError.describe(),
                leftRightDelta.describe(),
                battery.describe()));
        out.append(String.format(Locale.US,
                "  suspiciousCounts zeroTargetDropouts=%d speedCollapses=%d unexpectedInterrupts=%d\n",
                totalDropouts,
                totalCollapses,
                totalUnexpectedInterrupts));

        out.append("perMatch\n");
        for (MatchAnalysis analysis : analyses) {
            appendMatchReport(out, analysis);
        }

        out.append("conclusion\n");
        if (totalBrownouts == 0
                && totalMismatch <= LOOP_EPSILON_SEC
                && totalDropouts == 0
                && totalCollapses == 0
                && totalUnexpectedInterrupts == 0
                && steadyAvgError.p95() <= 110.0
                && steadyHoodError.p95() <= 0.80
                && leftRightDelta.p95() <= 90.0) {
            out.append("  Shooter behavior looks clean across q17+ logs: no brownouts, no command/target dropouts, no gate/kicker coherence issues, and steady-state wheel/hood tracking stayed tight match-to-match.\n");
        } else {
            out.append("  Review notable events below; at least one coherence or tracking threshold tripped.\n");
        }
        return out.toString();
    }

    private static void appendMatchReport(StringBuilder out, MatchAnalysis analysis) {
        out.append(String.format(Locale.US, "  %s file=%s\n", analysis.label(), analysis.fileName()));
        out.append(String.format(Locale.US,
                "    enabled=%.1fs auto=%.1fs teleop=%.1fs shooterActive=%.1fs highTarget=%.1fs selectedShoot=%.1fs aimOnly=%.1fs gateOpen=%.1fs kicker=%.1fs\n",
                analysis.enabledSec(),
                analysis.autoSec(),
                analysis.teleopSec(),
                analysis.shooterActiveSec(),
                analysis.highTargetSec(),
                analysis.selectedShootSec(),
                analysis.aimOnlySec(),
                analysis.gateOpenSec(),
                analysis.kickerActiveSec()));
        out.append(String.format(Locale.US,
                "    battery[minActive=%.2fV] totalCurrent[max=%.1fA] temps[left=%.1fC right=%.1fC hood=%.1fC kicker=%.1fC] brownouts=%d\n",
                analysis.minBatteryDuringShooterActive(),
                analysis.maxTotalCurrentAmps(),
                analysis.maxLeftTempC(),
                analysis.maxRightTempC(),
                analysis.maxHoodTempC(),
                analysis.maxKickerTempC(),
                analysis.brownoutCount()));
        out.append(String.format(Locale.US,
                "    steady avgErr=%s hoodErr=%s leftRightDelta=%s atSetpointDuty=%.1f%% readyDuty=%.1f%% gateWhileAtSetpoint=%.1f%% mismatchSec=%.3f\n",
                describeList(analysis.steadyAvgErrorRpmSamples()),
                describeList(analysis.steadyHoodErrorDegSamples()),
                describeList(analysis.steadyLeftRightVelocityDeltaSamples()),
                analysis.highTargetSec() <= 0.0 ? 0.0 : 100.0 * analysis.atSetpointWhileHighTargetSec() / analysis.highTargetSec(),
                analysis.highTargetSec() <= 0.0 ? 0.0 : 100.0 * analysis.readyWhileHighTargetSec() / analysis.highTargetSec(),
                analysis.gateOpenSec() <= 0.0 ? 0.0 : 100.0 * analysis.gateOpenWhileCoordinatorReadySec() / analysis.gateOpenSec(),
                analysis.gateKickerMismatchSec()));
        out.append("    commandSummary").append('\n');
        if (analysis.commandStats().isEmpty()) {
            out.append("      <none>\n");
        } else {
            for (CommandStats stats : analysis.commandStats().values()) {
                out.append(String.format(Locale.US,
                        "      %s starts=%d finishes=%d interrupts=%d meanDur=%.2fs maxDur=%.2fs\n",
                        stats.name,
                        stats.starts,
                        stats.finishes,
                        stats.interrupts,
                        stats.meanDurationSec(),
                        stats.maxDurationSec));
            }
        }
        out.append("    highTargetWindows\n");
        if (analysis.highTargetWindows().isEmpty()) {
            out.append("      <none>\n");
        } else {
            for (HighTargetWindow window : analysis.highTargetWindows()) {
                out.append(String.format(Locale.US,
                        "      %.3f-%.3f dur=%.2fs target=[%.0f,%.0f] spinup=%s atSetpoint=%.1f%% ready=%.1f%% gate=%.1f%% maxAvgErr=%.1f maxHoodErr=%.2f minBattery=%.2f state=%s\n",
                        window.startSec(),
                        window.endSec(),
                        window.durationSec(),
                        window.minTargetAvgRpm(),
                        window.maxTargetAvgRpm(),
                        finiteOr(window.spinupSec(), "<no-atSetpoint>"),
                        window.durationSec() <= 0.0 ? 0.0 : 100.0 * window.atSetpointSec() / window.durationSec(),
                        window.durationSec() <= 0.0 ? 0.0 : 100.0 * window.readySec() / window.durationSec(),
                        window.durationSec() <= 0.0 ? 0.0 : 100.0 * window.gateOpenSec() / window.durationSec(),
                        window.maxAbsAvgErrorRpm(),
                        window.maxAbsHoodErrorDeg(),
                        window.minBatteryVoltage(),
                        window.dominantState()));
            }
        }
        out.append("    notable\n");
        List<String> notable = new ArrayList<>();
        notable.addAll(analysis.zeroTargetDropouts());
        notable.addAll(analysis.speedCollapseEvents());
        notable.addAll(analysis.coherenceEvents());
        notable.addAll(analysis.unexpectedInterrupts());
        if (analysis.parseWarning() != null && !analysis.parseWarning().isBlank()) {
            notable.add("parseWarning: " + analysis.parseWarning());
        }
        if (!analysis.missingCriticalSignals().isEmpty()) {
            notable.add("missingCriticalSignals: " + String.join(", ", analysis.missingCriticalSignals()));
        }
        if (notable.isEmpty()) {
            out.append("      <none>\n");
        } else {
            for (String line : notable) {
                out.append("      ").append(line).append('\n');
            }
        }
    }

    private static String finiteOr(double value, String fallback) {
        return Double.isFinite(value) ? String.format(Locale.US, "%.2fs", value) : fallback;
    }

    private static String describeList(List<Double> values) {
        return new SummaryAccumulator(values).describe();
    }

    private static String buildHighTargetWindowCsv(List<MatchAnalysis> analyses) {
        StringBuilder out = new StringBuilder();
        out.append("match,startSec,endSec,durationSec,minTargetAvgRpm,maxTargetAvgRpm,spinupSec,atSetpointPct,readyPct,gatePct,maxAbsAvgErrRpm,maxAbsHoodErrDeg,minBatteryVoltage,dominantState\n");
        for (MatchAnalysis analysis : analyses) {
            for (HighTargetWindow window : analysis.highTargetWindows()) {
                out.append(String.format(Locale.US,
                        "%s,%.3f,%.3f,%.3f,%.1f,%.1f,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s\n",
                        analysis.label(),
                        window.startSec(),
                        window.endSec(),
                        window.durationSec(),
                        window.minTargetAvgRpm(),
                        window.maxTargetAvgRpm(),
                        Double.isFinite(window.spinupSec()) ? String.format(Locale.US, "%.3f", window.spinupSec()) : "",
                        window.durationSec() <= 0.0 ? 0.0 : 100.0 * window.atSetpointSec() / window.durationSec(),
                        window.durationSec() <= 0.0 ? 0.0 : 100.0 * window.readySec() / window.durationSec(),
                        window.durationSec() <= 0.0 ? 0.0 : 100.0 * window.gateOpenSec() / window.durationSec(),
                        window.maxAbsAvgErrorRpm(),
                        window.maxAbsHoodErrorDeg(),
                        window.minBatteryVoltage(),
                        csv(window.dominantState())));
            }
        }
        return out.toString();
    }

    private static String buildCommandEventCsv(List<MatchAnalysis> analyses) {
        StringBuilder out = new StringBuilder();
        out.append("match,timeSec,type,name,source,requirements,runId,interrupted,durationSec,snapshot\n");
        for (MatchAnalysis analysis : analyses) {
            for (CommandEvent event : analysis.commandEvents()) {
                out.append(String.format(Locale.US,
                        "%s,%.3f,%s,%s,%s,%s,%d,%s,%s,%s\n",
                        analysis.label(),
                        event.timestampSec(),
                        csv(event.type()),
                        csv(event.name()),
                        csv(event.source()),
                        csv(event.requirements()),
                        event.runId(),
                        event.interrupted(),
                        Double.isFinite(event.durationSec()) ? String.format(Locale.US, "%.3f", event.durationSec()) : "",
                        csv(event.snapshot())));
            }
        }
        return out.toString();
    }

    private static String csv(String value) {
        String safe = value == null ? "" : value;
        if (safe.contains(",") || safe.contains("\"") || safe.contains("\n")) {
            return '"' + safe.replace("\"", "\"\"") + '"';
        }
        return safe;
    }

    private record EntryInfo(String name, String type) {}

    private record CommandEvent(
            long timestampUs,
            String type,
            String name,
            String source,
            String requirements,
            long runId,
            boolean interrupted,
            double durationSec,
            String snapshot) {
        private double timestampSec() {
            return timestampUs / 1_000_000.0;
        }
    }

    private record HighTargetWindow(
            double startSec,
            double endSec,
            double durationSec,
            double minTargetAvgRpm,
            double maxTargetAvgRpm,
            double spinupSec,
            double atSetpointSec,
            double readySec,
            double gateOpenSec,
            double maxAbsAvgErrorRpm,
            double maxAbsHoodErrorDeg,
            double minBatteryVoltage,
            String dominantState) {}

    private record MatchAnalysis(
            String label,
            String fileName,
            double enabledSec,
            double autoSec,
            double teleopSec,
            double shooterActiveSec,
            double highTargetSec,
            double selectedShootSec,
            double aimOnlySec,
            double gateOpenSec,
            double kickerActiveSec,
            double atSetpointWhileHighTargetSec,
            double readyWhileHighTargetSec,
            double gateOpenWhileCoordinatorReadySec,
            double gateKickerMismatchSec,
            double minBatteryDuringShooterActive,
            double maxTotalCurrentAmps,
            double maxLeftTempC,
            double maxRightTempC,
            double maxHoodTempC,
            double maxKickerTempC,
            int brownoutCount,
            List<Double> steadyAvgErrorRpmSamples,
            List<Double> steadyHoodErrorDegSamples,
            List<Double> steadyLeftRightVelocityDeltaSamples,
            List<Double> shooterActiveBatterySamples,
            List<HighTargetWindow> highTargetWindows,
            Map<String, CommandStats> commandStats,
            List<String> zeroTargetDropouts,
            List<String> speedCollapseEvents,
            List<String> coherenceEvents,
            List<String> unexpectedInterrupts,
            List<String> missingCriticalSignals,
            List<CommandEvent> commandEvents,
            String parseWarning) {}

    private static final class CommandStats {
        private final String name;
        private int starts;
        private int finishes;
        private int interrupts;
        private double durationSecSum;
        private double maxDurationSec;

        private CommandStats(String name) {
            this.name = name;
        }

        private double meanDurationSec() {
            int count = finishes + interrupts;
            return count == 0 ? 0.0 : durationSecSum / count;
        }
    }

    private static final class TimelineState {
        private final Map<String, Boolean> commandRunning = new HashMap<>();
        private String mode = "";
        private boolean enabled = false;
        private boolean brownedOut = false;
        private double matchTimeSec = Double.NaN;
        private double batteryVoltage = Double.NaN;
        private double totalCurrentAmps = Double.NaN;

        private double targetLeftRpm = 0.0;
        private double targetRightRpm = 0.0;
        private double targetHoodDeg = 0.0;
        private double kickerTorqueCommandAmps = 0.0;
        private double kickerVoltageCommand = 0.0;
        private double leftVelocityErrorRpm = Double.NaN;
        private double rightVelocityErrorRpm = Double.NaN;
        private double hoodAngleErrorDeg = Double.NaN;
        private boolean leftVelocityAtSetpoint = false;
        private boolean rightVelocityAtSetpoint = false;
        private boolean hoodAtSetpoint = false;
        private boolean shooterAtSetpoint = false;
        private boolean readyToFire = false;
        private String readinessMode = "";
        private double readinessToleranceRpm = Double.NaN;

        private double leftVelocityRpm = Double.NaN;
        private double rightVelocityRpm = Double.NaN;
        private double leftAppliedVolts = Double.NaN;
        private double rightAppliedVolts = Double.NaN;
        private double leftSupplyCurrentAmps = Double.NaN;
        private double rightSupplyCurrentAmps = Double.NaN;
        private double leftTempC = Double.NaN;
        private double rightTempC = Double.NaN;
        private double hoodPositionDeg = Double.NaN;
        private double hoodVelocityRpm = Double.NaN;
        private double hoodAppliedVolts = Double.NaN;
        private double hoodSupplyCurrentAmps = Double.NaN;
        private double hoodStatorCurrentAmps = Double.NaN;
        private double hoodTempC = Double.NaN;
        private double kickerVelocityRpm = Double.NaN;
        private double kickerAppliedVolts = Double.NaN;
        private double kickerSupplyCurrentAmps = Double.NaN;
        private double kickerTempC = Double.NaN;

        private boolean shootingShooterAtSetpoint = false;
        private boolean aimReady = false;
        private boolean gateOpen = false;
        private String shootingState = "";
        private String blockReason = "";
        private boolean automaticFeedEnabled = false;
        private boolean manualFeedOverride = false;

        private String lastStartedName = "";
        private String lastStartedSource = "";
        private String lastStartedRequirements = "";
        private String lastEndedName = "";
        private String lastEndedSource = "";
        private boolean lastEndedInterrupted = false;
        private double lastEndedDurationSec = Double.NaN;

        private TimelineState(Map<Integer, String> knownCommandEntries) {
            for (String commandName : knownCommandEntries.values()) {
                commandRunning.put(commandName, false);
            }
        }

        private double averageTargetRpm() {
            return (targetLeftRpm + targetRightRpm) / 2.0;
        }

        private double averageMeasuredRpm() {
            return (leftVelocityRpm + rightVelocityRpm) / 2.0;
        }

        private double leftRightVelocityDeltaRpm() {
            return leftVelocityRpm - rightVelocityRpm;
        }

        private boolean kickerActive() {
            return Math.abs(kickerTorqueCommandAmps) > KICKER_ACTIVE_TORQUE_AMPS
                    || Math.abs(kickerVoltageCommand) > KICKER_ACTIVE_VOLTS
                    || Math.abs(kickerAppliedVolts) > KICKER_ACTIVE_VOLTS;
        }

        private boolean running(String commandName) {
            return commandRunning.getOrDefault(commandName, false);
        }

        private boolean anyShooterCommandRunning() {
            for (Map.Entry<String, Boolean> entry : commandRunning.entrySet()) {
                if (entry.getValue() && isRelevantShooterCommand(entry.getKey())) {
                    return true;
                }
            }
            return false;
        }

        private String snapshot() {
            return String.format(Locale.US,
                    "mode=%s enabled=%s matchTime=%.1f targetAvg=%.0f measuredAvg=%.0f hood=%.2f/%.2f gate=%s kicker=%s shootAt=%s ready=%s state=%s block=%s",
                    mode,
                    enabled,
                    matchTimeSec,
                    averageTargetRpm(),
                    averageMeasuredRpm(),
                    hoodPositionDeg,
                    targetHoodDeg,
                    gateOpen,
                    kickerActive(),
                    shootingShooterAtSetpoint,
                    readyToFire,
                    shootingState,
                    blockReason);
        }
    }

    private static boolean isRelevantShooterCommand(String name) {
        if (name == null || name.isBlank()) {
            return false;
        }
        String lower = name.toLowerCase(Locale.ROOT);
        return lower.contains("shooter") || lower.equals("stopmanipulators");
    }

    private static boolean looksLikeOperatorOrDefaultSource(String source) {
        if (source == null || source.isBlank()) {
            return false;
        }
        String lower = source.toLowerCase(Locale.ROOT);
        return lower.contains("driver.")
                || lower.contains("default.")
                || lower.contains("dashboard.")
                || lower.contains("teleopinit")
                || lower.contains("autonomousinit")
                || lower.contains("disabledinit")
                || lower.contains("testinit");
    }

    private static final class MatchStats {
        private final String label;
        private final EntryIds entryIds;
        private final Map<Integer, EntryInfo> entries;
        private final Path wpilog;

        private double enabledSec;
        private double autoSec;
        private double teleopSec;
        private double shooterActiveSec;
        private double highTargetSec;
        private double selectedShootSec;
        private double aimOnlySec;
        private double gateOpenSec;
        private double kickerActiveSec;
        private double atSetpointWhileHighTargetSec;
        private double readyWhileHighTargetSec;
        private double gateOpenWhileCoordinatorReadySec;
        private double gateKickerMismatchSec;
        private double minBatteryDuringShooterActive = Double.POSITIVE_INFINITY;
        private double maxTotalCurrentAmps = Double.NaN;
        private double maxLeftTempC = Double.NaN;
        private double maxRightTempC = Double.NaN;
        private double maxHoodTempC = Double.NaN;
        private double maxKickerTempC = Double.NaN;
        private int brownoutCount;

        private final List<Double> steadyAvgErrorRpmSamples = new ArrayList<>();
        private final List<Double> steadyHoodErrorDegSamples = new ArrayList<>();
        private final List<Double> steadyLeftRightVelocityDeltaSamples = new ArrayList<>();
        private final List<Double> shooterActiveBatterySamples = new ArrayList<>();
        private final List<HighTargetWindow> highTargetWindows = new ArrayList<>();
        private final LinkedHashMap<String, CommandStats> commandStats = new LinkedHashMap<>();
        private final List<String> zeroTargetDropouts = new ArrayList<>();
        private final List<String> speedCollapseEvents = new ArrayList<>();
        private final List<String> coherenceEvents = new ArrayList<>();
        private final List<String> unexpectedInterrupts = new ArrayList<>();
        private final List<CommandEvent> commandEvents = new ArrayList<>();
        private final Map<Long, CommandEvent> runningCommandStarts = new HashMap<>();
        private final List<String> missingCriticalSignals = new ArrayList<>();

        private HighWindowState currentHighWindow;
        private long zeroTargetWhileShootStartUs = Long.MIN_VALUE;
        private long zeroTargetWhileAimStartUs = Long.MIN_VALUE;
        private long speedCollapseStartUs = Long.MIN_VALUE;
        private long gateNoKickerStartUs = Long.MIN_VALUE;
        private long kickerNoGateStartUs = Long.MIN_VALUE;
        private long gateWithoutReadyStartUs = Long.MIN_VALUE;
        private boolean previousBrownoutState = false;

        private MatchStats(String label, EntryIds entryIds, Map<Integer, EntryInfo> entries, Path wpilog) {
            this.label = label;
            this.entryIds = entryIds;
            this.entries = entries;
            this.wpilog = wpilog;
            entryIds.appendMissingCriticalSignals(missingCriticalSignals);
        }

        private void flushSample(long sampleTimestampUs, long nextTimestampUs, TimelineState state) {
            double dtSec = Math.max(0.0, (nextTimestampUs - sampleTimestampUs) / 1_000_000.0);
            if (dtSec <= 0.0) {
                return;
            }
            if (dtSec > 0.50) {
                finalizeHighWindow(sampleTimestampUs, state);
                zeroTargetWhileShootStartUs = Long.MIN_VALUE;
                zeroTargetWhileAimStartUs = Long.MIN_VALUE;
                speedCollapseStartUs = Long.MIN_VALUE;
                gateNoKickerStartUs = Long.MIN_VALUE;
                kickerNoGateStartUs = Long.MIN_VALUE;
                gateWithoutReadyStartUs = Long.MIN_VALUE;
                return;
            }

            boolean highTarget = state.enabled && Math.abs(state.averageTargetRpm()) >= HIGH_TARGET_THRESHOLD_RPM;
            boolean selectedShoot = state.enabled && state.running("ShooterSelectedShootMode");
            boolean aimOnly = state.enabled && state.running("ShooterDriverAim");
            boolean home = state.enabled && state.running("ShooterHome");
            boolean kickerActive = state.kickerActive();
            boolean shooterActive = state.enabled
                    && (Math.abs(state.averageTargetRpm()) >= TARGET_PRESENT_THRESHOLD_RPM
                            || selectedShoot
                            || aimOnly
                            || home
                            || kickerActive
                            || state.anyShooterCommandRunning());

            if (state.enabled) {
                enabledSec += dtSec;
                if ("AUTONOMOUS".equalsIgnoreCase(state.mode)) {
                    autoSec += dtSec;
                }
                if ("TELEOP".equalsIgnoreCase(state.mode)) {
                    teleopSec += dtSec;
                }
            }
            if (shooterActive) {
                shooterActiveSec += dtSec;
                if (Double.isFinite(state.batteryVoltage)) {
                    minBatteryDuringShooterActive = Math.min(minBatteryDuringShooterActive, state.batteryVoltage);
                    shooterActiveBatterySamples.add(state.batteryVoltage);
                }
            }
            if (highTarget) {
                highTargetSec += dtSec;
            }
            if (selectedShoot) {
                selectedShootSec += dtSec;
            }
            if (aimOnly) {
                aimOnlySec += dtSec;
            }
            if (state.gateOpen) {
                gateOpenSec += dtSec;
            }
            if (kickerActive) {
                kickerActiveSec += dtSec;
            }
            if (highTarget && state.shooterAtSetpoint) {
                atSetpointWhileHighTargetSec += dtSec;
            }
            if (highTarget && state.readyToFire) {
                readyWhileHighTargetSec += dtSec;
            }
            if (state.gateOpen && state.shootingShooterAtSetpoint && state.aimReady) {
                gateOpenWhileCoordinatorReadySec += dtSec;
            }
            if (state.gateOpen != kickerActive) {
                gateKickerMismatchSec += dtSec;
            }
            if (state.brownedOut && !previousBrownoutState) {
                brownoutCount++;
            }
            previousBrownoutState = state.brownedOut;

            maxTotalCurrentAmps = maxFinite(maxTotalCurrentAmps, state.totalCurrentAmps);
            maxLeftTempC = maxFinite(maxLeftTempC, state.leftTempC);
            maxRightTempC = maxFinite(maxRightTempC, state.rightTempC);
            maxHoodTempC = maxFinite(maxHoodTempC, state.hoodTempC);
            maxKickerTempC = maxFinite(maxKickerTempC, state.kickerTempC);

            updateHighTargetWindow(sampleTimestampUs, nextTimestampUs, dtSec, state, highTarget);
            trackZeroTargetDropouts(sampleTimestampUs, nextTimestampUs, state, selectedShoot, aimOnly);
            trackCoherence(sampleTimestampUs, nextTimestampUs, state, kickerActive);
        }

        private void finish(long lastTimestampUs, TimelineState state) {
            finalizeHighWindow(lastTimestampUs, state);
            closeStreakIfNeeded(zeroTargetWhileShootStartUs, lastTimestampUs, ZERO_TARGET_STREAK_THRESHOLD_SEC,
                    zeroTargetDropouts,
                    String.format(Locale.US,
                            "%.3fs zero-target dropout while ShooterSelectedShootMode running (%s)",
                            lastTimestampUs / 1_000_000.0,
                            state.snapshot()));
            closeStreakIfNeeded(zeroTargetWhileAimStartUs, lastTimestampUs, ZERO_TARGET_STREAK_THRESHOLD_SEC,
                    zeroTargetDropouts,
                    String.format(Locale.US,
                            "%.3fs zero-target dropout while ShooterDriverAim running (%s)",
                            lastTimestampUs / 1_000_000.0,
                            state.snapshot()));
            closeStreakIfNeeded(speedCollapseStartUs, lastTimestampUs, SPEED_COLLAPSE_STREAK_THRESHOLD_SEC,
                    speedCollapseEvents,
                    String.format(Locale.US,
                            "%.3fs sustained speed collapse near target (%s)",
                            lastTimestampUs / 1_000_000.0,
                            state.snapshot()));
            closeStreakIfNeeded(gateNoKickerStartUs, lastTimestampUs, COHERENCE_STREAK_THRESHOLD_SEC,
                    coherenceEvents,
                    String.format(Locale.US,
                            "%.3fs gate open without kicker output (%s)",
                            lastTimestampUs / 1_000_000.0,
                            state.snapshot()));
            closeStreakIfNeeded(kickerNoGateStartUs, lastTimestampUs, COHERENCE_STREAK_THRESHOLD_SEC,
                    coherenceEvents,
                    String.format(Locale.US,
                            "%.3fs kicker active while gate closed (%s)",
                            lastTimestampUs / 1_000_000.0,
                            state.snapshot()));
            closeStreakIfNeeded(gateWithoutReadyStartUs, lastTimestampUs, COHERENCE_STREAK_THRESHOLD_SEC,
                    coherenceEvents,
                    String.format(Locale.US,
                            "%.3fs gate open while coordinator not ready (%s)",
                            lastTimestampUs / 1_000_000.0,
                            state.snapshot()));
        }

        private void recordCommandEvent(CommandEvent event) {
            commandEvents.add(event);
            if (!isRelevantShooterCommand(event.name()) && !event.requirements().toLowerCase(Locale.ROOT).contains("shooter")) {
                return;
            }
            CommandStats stats = commandStats.computeIfAbsent(event.name(), CommandStats::new);
            if ("START".equals(event.type())) {
                stats.starts++;
                runningCommandStarts.put(event.runId(), event);
                return;
            }

            if (event.interrupted()) {
                stats.interrupts++;
            } else {
                stats.finishes++;
            }
            if (Double.isFinite(event.durationSec())) {
                stats.durationSecSum += event.durationSec();
                stats.maxDurationSec = Math.max(stats.maxDurationSec, event.durationSec());
            }

            if (event.interrupted()) {
                CommandEvent start = runningCommandStarts.get(event.runId());
                if (!classifyInterruptAsExpected(event, start)) {
                    addLimited(unexpectedInterrupts,
                            String.format(Locale.US,
                                    "%.3fs unexpected interrupt name=%s run=%d duration=%.3fs source=%s snapshot=%s",
                                    event.timestampSec(),
                                    event.name(),
                                    event.runId(),
                                    event.durationSec(),
                                    event.source(),
                                    event.snapshot()));
                }
            }
            runningCommandStarts.remove(event.runId());
        }

        private boolean classifyInterruptAsExpected(CommandEvent end, CommandEvent start) {
            double tsSec = end.timestampSec();
            if (looksLikeOperatorOrDefaultSource(end.source())
                    || (start != null && looksLikeOperatorOrDefaultSource(start.source()))) {
                return true;
            }
            for (CommandEvent other : commandEvents) {
                if (other == end || !"START".equals(other.type())) {
                    continue;
                }
                if (Math.abs(other.timestampSec() - tsSec) <= 0.20 && isRelevantShooterCommand(other.name())) {
                    return true;
                }
            }
            return false;
        }

        private void updateHighTargetWindow(long sampleTimestampUs, long nextTimestampUs, double dtSec, TimelineState state, boolean highTarget) {
            if (!highTarget) {
                finalizeHighWindow(sampleTimestampUs, state);
                return;
            }
            if (currentHighWindow == null) {
                currentHighWindow = new HighWindowState(sampleTimestampUs, state);
            }
            currentHighWindow.endUs = nextTimestampUs;
            currentHighWindow.durationSec += dtSec;
            currentHighWindow.minTargetAvgRpm = Math.min(currentHighWindow.minTargetAvgRpm, Math.abs(state.averageTargetRpm()));
            currentHighWindow.maxTargetAvgRpm = Math.max(currentHighWindow.maxTargetAvgRpm, Math.abs(state.averageTargetRpm()));
            currentHighWindow.maxAbsAvgErrorRpm = Math.max(currentHighWindow.maxAbsAvgErrorRpm,
                    absFinite(state.averageTargetRpm() - state.averageMeasuredRpm()));
            currentHighWindow.maxAbsHoodErrorDeg = Math.max(currentHighWindow.maxAbsHoodErrorDeg,
                    absFinite(state.targetHoodDeg - state.hoodPositionDeg));
            currentHighWindow.minBatteryVoltage = minFinite(currentHighWindow.minBatteryVoltage, state.batteryVoltage);
            currentHighWindow.stateTime.merge(blankToUnknown(state.shootingState), dtSec, Double::sum);

            if (state.shooterAtSetpoint) {
                currentHighWindow.atSetpointSec += dtSec;
                if (currentHighWindow.firstAtSetpointUs == Long.MIN_VALUE) {
                    currentHighWindow.firstAtSetpointUs = sampleTimestampUs;
                }
            }
            if (state.readyToFire) {
                currentHighWindow.readySec += dtSec;
            }
            if (state.gateOpen) {
                currentHighWindow.gateOpenSec += dtSec;
            }

            if (currentHighWindow.firstAtSetpointUs != Long.MIN_VALUE) {
                steadyAvgErrorRpmSamples.add(absFinite(state.averageTargetRpm() - state.averageMeasuredRpm()));
                steadyHoodErrorDegSamples.add(absFinite(state.targetHoodDeg - state.hoodPositionDeg));
                steadyLeftRightVelocityDeltaSamples.add(absFinite(state.leftRightVelocityDeltaRpm()));
            }

            boolean collapse = currentHighWindow.firstAtSetpointUs != Long.MIN_VALUE
                    && Math.abs(state.averageMeasuredRpm()) < 0.8 * Math.abs(state.averageTargetRpm())
                    && !state.brownedOut;
            speedCollapseStartUs = updateStreak(
                    speedCollapseStartUs,
                    collapse,
                    sampleTimestampUs,
                    nextTimestampUs,
                    SPEED_COLLAPSE_STREAK_THRESHOLD_SEC,
                    speedCollapseEvents,
                    String.format(Locale.US,
                            "%.3fs sustained speed collapse targetAvg=%.0f measuredAvg=%.0f battery=%.2f state=%s",
                            sampleTimestampUs / 1_000_000.0,
                            state.averageTargetRpm(),
                            state.averageMeasuredRpm(),
                            state.batteryVoltage,
                            state.snapshot()));
        }

        private void finalizeHighWindow(long timestampUs, TimelineState state) {
            if (currentHighWindow == null) {
                return;
            }
            HighWindowState window = currentHighWindow;
            currentHighWindow = null;
            highTargetWindows.add(new HighTargetWindow(
                    window.startUs / 1_000_000.0,
                    Math.max(window.endUs, timestampUs) / 1_000_000.0,
                    window.durationSec,
                    window.minTargetAvgRpm,
                    window.maxTargetAvgRpm,
                    window.firstAtSetpointUs == Long.MIN_VALUE
                            ? Double.NaN
                            : (window.firstAtSetpointUs - window.startUs) / 1_000_000.0,
                    window.atSetpointSec,
                    window.readySec,
                    window.gateOpenSec,
                    window.maxAbsAvgErrorRpm,
                    window.maxAbsHoodErrorDeg,
                    finiteOrDefault(window.minBatteryVoltage, Double.NaN),
                    dominantState(window.stateTime)));
            speedCollapseStartUs = Long.MIN_VALUE;
        }

        private void trackZeroTargetDropouts(long sampleTimestampUs, long nextTimestampUs, TimelineState state, boolean selectedShoot, boolean aimOnly) {
            boolean zeroTarget = Math.abs(state.averageTargetRpm()) < TARGET_PRESENT_THRESHOLD_RPM;
            zeroTargetWhileShootStartUs = updateStreak(
                    zeroTargetWhileShootStartUs,
                    selectedShoot && zeroTarget,
                    sampleTimestampUs,
                    nextTimestampUs,
                    ZERO_TARGET_STREAK_THRESHOLD_SEC,
                    zeroTargetDropouts,
                    String.format(Locale.US,
                            "%.3fs zero target while ShooterSelectedShootMode active targetAvg=%.0f state=%s",
                            sampleTimestampUs / 1_000_000.0,
                            state.averageTargetRpm(),
                            state.snapshot()));
            zeroTargetWhileAimStartUs = updateStreak(
                    zeroTargetWhileAimStartUs,
                    aimOnly && zeroTarget,
                    sampleTimestampUs,
                    nextTimestampUs,
                    ZERO_TARGET_STREAK_THRESHOLD_SEC,
                    zeroTargetDropouts,
                    String.format(Locale.US,
                            "%.3fs zero target while ShooterDriverAim active targetAvg=%.0f state=%s",
                            sampleTimestampUs / 1_000_000.0,
                            state.averageTargetRpm(),
                            state.snapshot()));
        }

        private void trackCoherence(long sampleTimestampUs, long nextTimestampUs, TimelineState state, boolean kickerActive) {
            gateNoKickerStartUs = updateStreak(
                    gateNoKickerStartUs,
                    state.gateOpen && !kickerActive,
                    sampleTimestampUs,
                    nextTimestampUs,
                    COHERENCE_STREAK_THRESHOLD_SEC,
                    coherenceEvents,
                    String.format(Locale.US,
                            "%.3fs gate open without kicker output state=%s",
                            sampleTimestampUs / 1_000_000.0,
                            state.snapshot()));
            kickerNoGateStartUs = updateStreak(
                    kickerNoGateStartUs,
                    kickerActive && !state.gateOpen && state.automaticFeedEnabled,
                    sampleTimestampUs,
                    nextTimestampUs,
                    COHERENCE_STREAK_THRESHOLD_SEC,
                    coherenceEvents,
                    String.format(Locale.US,
                            "%.3fs kicker active while gate closed state=%s",
                            sampleTimestampUs / 1_000_000.0,
                            state.snapshot()));
            gateWithoutReadyStartUs = updateStreak(
                    gateWithoutReadyStartUs,
                    state.gateOpen
                            && !state.manualFeedOverride
                            && !(state.shootingShooterAtSetpoint && state.aimReady),
                    sampleTimestampUs,
                    nextTimestampUs,
                    COHERENCE_STREAK_THRESHOLD_SEC,
                    coherenceEvents,
                    String.format(Locale.US,
                            "%.3fs gate open while not ready aimReady=%s shootAt=%s state=%s",
                            sampleTimestampUs / 1_000_000.0,
                            state.aimReady,
                            state.shootingShooterAtSetpoint,
                            state.snapshot()));
        }

        private long updateStreak(
                long startUs,
                boolean condition,
                long sampleTimestampUs,
                long nextTimestampUs,
                double thresholdSec,
                List<String> sink,
                String message) {
            if (condition) {
                return startUs == Long.MIN_VALUE ? sampleTimestampUs : startUs;
            }
            if (startUs != Long.MIN_VALUE && (sampleTimestampUs - startUs) / 1_000_000.0 >= thresholdSec) {
                addLimited(sink, message);
            }
            return Long.MIN_VALUE;
        }

        private void closeStreakIfNeeded(long startUs, long endUs, double thresholdSec, List<String> sink, String message) {
            if (startUs != Long.MIN_VALUE && (endUs - startUs) / 1_000_000.0 >= thresholdSec) {
                addLimited(sink, message);
            }
        }

        private MatchAnalysis toAnalysis(String parseWarning) {
            return new MatchAnalysis(
                    label,
                    wpilog.getFileName().toString(),
                    enabledSec,
                    autoSec,
                    teleopSec,
                    shooterActiveSec,
                    highTargetSec,
                    selectedShootSec,
                    aimOnlySec,
                    gateOpenSec,
                    kickerActiveSec,
                    atSetpointWhileHighTargetSec,
                    readyWhileHighTargetSec,
                    gateOpenWhileCoordinatorReadySec,
                    gateKickerMismatchSec,
                    finiteOrDefault(minBatteryDuringShooterActive, Double.NaN),
                    finiteOrDefault(maxTotalCurrentAmps, Double.NaN),
                    finiteOrDefault(maxLeftTempC, Double.NaN),
                    finiteOrDefault(maxRightTempC, Double.NaN),
                    finiteOrDefault(maxHoodTempC, Double.NaN),
                    finiteOrDefault(maxKickerTempC, Double.NaN),
                    brownoutCount,
                    List.copyOf(steadyAvgErrorRpmSamples),
                    List.copyOf(steadyHoodErrorDegSamples),
                    List.copyOf(steadyLeftRightVelocityDeltaSamples),
                    List.copyOf(shooterActiveBatterySamples),
                    List.copyOf(highTargetWindows),
                    Map.copyOf(commandStats),
                    List.copyOf(zeroTargetDropouts),
                    List.copyOf(speedCollapseEvents),
                    List.copyOf(coherenceEvents),
                    List.copyOf(unexpectedInterrupts),
                    List.copyOf(missingCriticalSignals),
                    List.copyOf(commandEvents),
                    parseWarning == null ? "" : parseWarning);
        }
    }

    private static final class HighWindowState {
        private final long startUs;
        private long endUs;
        private long firstAtSetpointUs = Long.MIN_VALUE;
        private double durationSec;
        private double minTargetAvgRpm;
        private double maxTargetAvgRpm;
        private double atSetpointSec;
        private double readySec;
        private double gateOpenSec;
        private double maxAbsAvgErrorRpm;
        private double maxAbsHoodErrorDeg;
        private double minBatteryVoltage = Double.POSITIVE_INFINITY;
        private final Map<String, Double> stateTime = new HashMap<>();

        private HighWindowState(long startUs, TimelineState state) {
            this.startUs = startUs;
            this.endUs = startUs;
            this.minTargetAvgRpm = Math.abs(state.averageTargetRpm());
            this.maxTargetAvgRpm = Math.abs(state.averageTargetRpm());
        }
    }

    private static final class EntryIds {
        private int modeEntry = -1;
        private int enabledEntry = -1;
        private int matchTimeEntry = -1;
        private int batteryVoltageEntry = -1;
        private int pdpTotalCurrentEntry = -1;
        private int brownoutEntry = -1;

        private int targetLeftRpmEntry = -1;
        private int targetRightRpmEntry = -1;
        private int targetHoodDegEntry = -1;
        private int kickerTorqueEntry = -1;
        private int kickerVoltageEntry = -1;
        private int readinessLeftErrorEntry = -1;
        private int readinessRightErrorEntry = -1;
        private int readinessHoodErrorEntry = -1;
        private int leftAtSetpointEntry = -1;
        private int rightAtSetpointEntry = -1;
        private int hoodAtSetpointEntry = -1;
        private int shooterAtSetpointEntry = -1;
        private int readyToFireEntry = -1;
        private int readinessModeEntry = -1;
        private int readinessToleranceEntry = -1;

        private int leftVelocityEntry = -1;
        private int rightVelocityEntry = -1;
        private int leftAppliedVoltsEntry = -1;
        private int rightAppliedVoltsEntry = -1;
        private int leftSupplyCurrentEntry = -1;
        private int rightSupplyCurrentEntry = -1;
        private int leftTempEntry = -1;
        private int rightTempEntry = -1;
        private int hoodPositionEntry = -1;
        private int hoodVelocityEntry = -1;
        private int hoodAppliedVoltsEntry = -1;
        private int hoodSupplyCurrentEntry = -1;
        private int hoodStatorCurrentEntry = -1;
        private int hoodTempEntry = -1;
        private int kickerVelocityEntry = -1;
        private int kickerAppliedVoltsEntry = -1;
        private int kickerSupplyCurrentEntry = -1;
        private int kickerTempEntry = -1;

        private int shootingShooterAtSetpointEntry = -1;
        private int aimReadyEntry = -1;
        private int gateOpenEntry = -1;
        private int shootingStateEntry = -1;
        private int blockReasonEntry = -1;
        private int automaticFeedEnabledEntry = -1;
        private int manualFeedOverrideEntry = -1;

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

        private void capture(int entry, String name, String type) {
            String lower = name.toLowerCase(Locale.ROOT);
            if (lower.contains("robotstate/mode")) {
                modeEntry = entry;
            } else if (lower.contains("robotstate/enabled")) {
                enabledEntry = entry;
            } else if (lower.contains("robotstate/matchtime")) {
                matchTimeEntry = entry;
            } else if (lower.endsWith("robotstate/batteryvoltage") || lower.endsWith("powerdistribution/totalvoltagevolts")) {
                batteryVoltageEntry = entry;
            } else if (lower.endsWith("powerdistribution/totalcurrentamps")) {
                pdpTotalCurrentEntry = entry;
            } else if (lower.contains("systemstats/brownedout")) {
                brownoutEntry = entry;
            } else if (lower.contains("shooter/targetleftrpm")) {
                targetLeftRpmEntry = entry;
            } else if (lower.contains("shooter/targetrightrpm")) {
                targetRightRpmEntry = entry;
            } else if (lower.contains("shooter/targethooddeg")) {
                targetHoodDegEntry = entry;
            } else if (lower.contains("shooter/kickertorqueamps")) {
                kickerTorqueEntry = entry;
            } else if (lower.contains("shooter/kickervoltage")) {
                kickerVoltageEntry = entry;
            } else if (lower.contains("shooter/readiness/leftvelocityerrorrpm")) {
                readinessLeftErrorEntry = entry;
            } else if (lower.contains("shooter/readiness/rightvelocityerrorrpm")) {
                readinessRightErrorEntry = entry;
            } else if (lower.contains("shooter/readiness/hoodangleerrordeg")) {
                readinessHoodErrorEntry = entry;
            } else if (lower.contains("shooter/readiness/leftvelocityatsetpoint")) {
                leftAtSetpointEntry = entry;
            } else if (lower.contains("shooter/readiness/rightvelocityatsetpoint")) {
                rightAtSetpointEntry = entry;
            } else if (lower.contains("shooter/readiness/hoodangleatsetpoint")) {
                hoodAtSetpointEntry = entry;
            } else if (lower.contains("shooter/atsetpoint")) {
                shooterAtSetpointEntry = entry;
            } else if (lower.contains("shooter/readytofire")) {
                readyToFireEntry = entry;
            } else if (lower.contains("shooter/readiness/mode")) {
                readinessModeEntry = entry;
            } else if (lower.contains("shooter/readiness/rpmtolerance")) {
                readinessToleranceEntry = entry;
            } else if (lower.contains("shooter/shooterleftvelocityrpm")) {
                leftVelocityEntry = entry;
            } else if (lower.contains("shooter/shooterrightvelocityrpm")) {
                rightVelocityEntry = entry;
            } else if (lower.contains("shooter/shooterleftappliedvolts")) {
                leftAppliedVoltsEntry = entry;
            } else if (lower.contains("shooter/shooterrightappliedvolts")) {
                rightAppliedVoltsEntry = entry;
            } else if (lower.contains("shooter/shooterleftsupplycurrentamps")) {
                leftSupplyCurrentEntry = entry;
            } else if (lower.contains("shooter/shooterrightsupplycurrentamps")) {
                rightSupplyCurrentEntry = entry;
            } else if (lower.contains("shooter/shooterlefttempcelsius")) {
                leftTempEntry = entry;
            } else if (lower.contains("shooter/shooterrighttempcelsius")) {
                rightTempEntry = entry;
            } else if (lower.contains("shooter/hoodpositionrad")) {
                hoodPositionEntry = entry;
            } else if (lower.contains("shooter/hoodvelocityrpm")) {
                hoodVelocityEntry = entry;
            } else if (lower.contains("shooter/hoodappliedvolts")) {
                hoodAppliedVoltsEntry = entry;
            } else if (lower.contains("shooter/hoodsupplycurrentamps")) {
                hoodSupplyCurrentEntry = entry;
            } else if (lower.contains("shooter/hoodstatorcurrentamps")) {
                hoodStatorCurrentEntry = entry;
            } else if (lower.contains("shooter/hoodtempcelsius")) {
                hoodTempEntry = entry;
            } else if (lower.contains("shooter/kickervelocityrpm")) {
                kickerVelocityEntry = entry;
            } else if (lower.contains("shooter/kickerappliedvolts")) {
                kickerAppliedVoltsEntry = entry;
            } else if (lower.contains("shooter/kickersupplycurrentamps")) {
                kickerSupplyCurrentEntry = entry;
            } else if (lower.contains("shooter/kickertempcelsius")) {
                kickerTempEntry = entry;
            } else if (lower.contains("shooting/shooteratsetpoint")) {
                shootingShooterAtSetpointEntry = entry;
            } else if (lower.contains("shooting/aimready")) {
                aimReadyEntry = entry;
            } else if (lower.contains("shooting/gateopen")) {
                gateOpenEntry = entry;
            } else if (lower.contains("shooting/state")) {
                shootingStateEntry = entry;
            } else if (lower.contains("shooting/blockreason")) {
                blockReasonEntry = entry;
            } else if (lower.contains("shooting/automaticfeedenabled")) {
                automaticFeedEnabledEntry = entry;
            } else if (lower.contains("shooting/manualfeedoverride")) {
                manualFeedOverrideEntry = entry;
            } else if (lower.contains("commands/laststarted/name")) {
                lastStartedNameEntry = entry;
            } else if (lower.contains("commands/laststarted/source")) {
                lastStartedSourceEntry = entry;
            } else if (lower.contains("commands/laststarted/requirements")) {
                lastStartedRequirementsEntry = entry;
            } else if (lower.contains("commands/laststarted/runid")) {
                lastStartedRunIdEntry = entry;
            } else if (lower.contains("commands/lastended/name")) {
                lastEndedNameEntry = entry;
            } else if (lower.contains("commands/lastended/source")) {
                lastEndedSourceEntry = entry;
            } else if (lower.contains("commands/lastended/interrupted")) {
                lastEndedInterruptedEntry = entry;
            } else if (lower.contains("commands/lastended/durationsec")) {
                lastEndedDurationEntry = entry;
            } else if (lower.contains("commands/lastended/runid")) {
                lastEndedRunIdEntry = entry;
            }

            String commandName = extractCommandName(name);
            if (commandName != null && isBooleanType(type)) {
                commandRunningEntries.put(entry, commandName);
            }
        }

        private void appendMissingCriticalSignals(List<String> sink) {
            if (modeEntry < 0) sink.add("RobotState/Mode");
            if (enabledEntry < 0) sink.add("RobotState/Enabled");
            if (batteryVoltageEntry < 0) sink.add("RobotState/BatteryVoltage");
            if (targetLeftRpmEntry < 0) sink.add("Shooter/TargetLeftRpm");
            if (targetRightRpmEntry < 0) sink.add("Shooter/TargetRightRpm");
            if (targetHoodDegEntry < 0) sink.add("Shooter/TargetHoodDeg");
            if (leftVelocityEntry < 0) sink.add("Shooter/shooterLeftVelocityRpm");
            if (rightVelocityEntry < 0) sink.add("Shooter/shooterRightVelocityRpm");
            if (hoodPositionEntry < 0) sink.add("Shooter/hoodPositionRad");
            if (shooterAtSetpointEntry < 0) sink.add("Shooter/AtSetpoint");
            if (readyToFireEntry < 0) sink.add("Shooter/ReadyToFire");
            if (gateOpenEntry < 0) sink.add("Shooting/GateOpen");
        }

        private static boolean isBooleanType(String type) {
            return type != null && type.toLowerCase(Locale.ROOT).contains("boolean");
        }

        private static String extractCommandName(String fullName) {
            String lower = fullName.toLowerCase(Locale.ROOT);
            int index = lower.lastIndexOf("commands/");
            if (index < 0) {
                return null;
            }
            String suffix = fullName.substring(index + "Commands/".length());
            if (suffix.isBlank()) {
                return null;
            }
            String lowerSuffix = suffix.toLowerCase(Locale.ROOT);
            if (lowerSuffix.startsWith("last") || lowerSuffix.startsWith("running/") || lowerSuffix.startsWith("byname/")
                    || lowerSuffix.startsWith("recentevents") || lowerSuffix.startsWith("running/")
                    || lowerSuffix.startsWith("lastschedule/") || lowerSuffix.startsWith("lastcancelallsource")) {
                return null;
            }
            if (suffix.contains("/")) {
                return null;
            }
            return suffix;
        }
    }

    private static final class SummaryAccumulator {
        private final List<Double> values = new ArrayList<>();

        private SummaryAccumulator() {}

        private SummaryAccumulator(List<Double> initial) {
            addAll(initial);
        }

        private void add(double value) {
            if (Double.isFinite(value)) {
                values.add(value);
            }
        }

        private void addAll(List<Double> other) {
            for (double value : other) {
                add(value);
            }
        }

        private double p95() {
            return percentile(0.95);
        }

        private String describe() {
            if (values.isEmpty()) {
                return "n=0";
            }
            List<Double> sorted = new ArrayList<>(values);
            sorted.sort(Double::compare);
            double sum = 0.0;
            for (double value : sorted) {
                sum += value;
            }
            return String.format(Locale.US,
                    "n=%d min=%.2f med=%.2f p95=%.2f max=%.2f mean=%.2f",
                    sorted.size(),
                    sorted.get(0),
                    percentile(sorted, 0.50),
                    percentile(sorted, 0.95),
                    sorted.get(sorted.size() - 1),
                    sum / sorted.size());
        }

        private double percentile(double fraction) {
            if (values.isEmpty()) {
                return Double.NaN;
            }
            List<Double> sorted = new ArrayList<>(values);
            sorted.sort(Double::compare);
            return percentile(sorted, fraction);
        }

        private static double percentile(List<Double> sorted, double fraction) {
            if (sorted.isEmpty()) {
                return Double.NaN;
            }
            int index = (int) Math.round(fraction * (sorted.size() - 1));
            index = Math.max(0, Math.min(sorted.size() - 1, index));
            return sorted.get(index);
        }
    }

    private static double maxFinite(double current, double candidate) {
        if (!Double.isFinite(candidate)) {
            return current;
        }
        if (!Double.isFinite(current)) {
            return candidate;
        }
        return Math.max(current, candidate);
    }

    private static double minFinite(double current, double candidate) {
        if (!Double.isFinite(candidate)) {
            return current;
        }
        if (!Double.isFinite(current)) {
            return candidate;
        }
        return Math.min(current, candidate);
    }

    private static double absFinite(double value) {
        return Double.isFinite(value) ? Math.abs(value) : Double.NaN;
    }

    private static double finiteOrDefault(double value, double fallback) {
        return Double.isFinite(value) ? value : fallback;
    }

    private static String dominantState(Map<String, Double> stateTime) {
        String best = "UNKNOWN";
        double bestTime = -1.0;
        for (Map.Entry<String, Double> entry : stateTime.entrySet()) {
            if (entry.getValue() > bestTime) {
                best = entry.getKey();
                bestTime = entry.getValue();
            }
        }
        return best;
    }

    private static String blankToUnknown(String value) {
        return value == null || value.isBlank() ? "UNKNOWN" : value;
    }

    private static void addLimited(List<String> sink, String line) {
        if (sink.size() < MAX_NOTABLE_EVENTS) {
            sink.add(line);
        }
    }
}
