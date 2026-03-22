package frc.robot.diagnostics;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Deque;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import org.junit.jupiter.api.Test;

class AutoAimHeadingLogAnalyzerTest {
    private static final Pattern QUALS_PATTERN = Pattern.compile(".*_q(\\d+)\\.wpilog$");
    private static final long FRESH_US = 60_000L;
    private static final long WINDOW_BREAK_US = 500_000L;
    private static final long MAX_PLAUSIBLE_TIMESTAMP_US = 1_000_000_000L;
    private static final double GAP_WARN_SEC = 0.060;
    private static final double GAP_BAD_SEC = 0.100;
    private static final double SHORT_RUN_WARN_SEC = 0.150;
    private static final double LARGE_ERROR_DEG = 3.0;
    private static final double LARGE_OMEGA_RAD_PER_SEC = 0.20;
    private static final double ZERO_COMMAND_RAD_PER_SEC = 0.05;
    private static final double READY_ERROR_MARGIN_DEG = 0.50;
    private static final Path DEFAULT_LOG_DIR = Path.of("logs", "mdbet");
    private static final int MIN_MATCH_EXCLUSIVE = 7;

    @Test
    void analyzeAutoAimHeadingAcrossMdbetAfterMatch7() throws IOException {
        Path logDir = DEFAULT_LOG_DIR.toAbsolutePath();
        assertTrue(Files.isDirectory(logDir), "Missing log directory: " + logDir);

        List<Path> logs = discoverLogs(logDir, MIN_MATCH_EXCLUSIVE);
        assertFalse(logs.isEmpty(), "No qualification wpilogs found after match " + MIN_MATCH_EXCLUSIVE + " in " + logDir);

        List<LogSummary> summaries = new ArrayList<>();
        for (Path log : logs) {
            summaries.add(analyzeLog(log));
        }

        String report = formatReport(logDir, summaries);
        Path out = Path.of("build", "reports", "diagnostics", "autoaim-heading-summary.txt").toAbsolutePath();
        Files.createDirectories(out.getParent());
        Files.writeString(out, report);
        System.out.println("Auto aim heading summary written: " + out);
    }

    private static List<Path> discoverLogs(Path logDir, int minMatchExclusive) throws IOException {
        try (var stream = Files.list(logDir)) {
            return stream.filter(Files::isRegularFile)
                    .filter(path -> {
                        Matcher matcher = QUALS_PATTERN.matcher(path.getFileName().toString());
                        return matcher.matches() && Integer.parseInt(matcher.group(1)) > minMatchExclusive;
                    })
                    .sorted(Comparator.comparingInt(AutoAimHeadingLogAnalyzerTest::matchNumber))
                    .collect(Collectors.toList());
        }
    }

    private static int matchNumber(Path path) {
        Matcher matcher = QUALS_PATTERN.matcher(path.getFileName().toString());
        if (!matcher.matches()) {
            return Integer.MIN_VALUE;
        }
        return Integer.parseInt(matcher.group(1));
    }

    private static LogSummary analyzeLog(Path wpilog) throws IOException {
        DataLogReader reader = new DataLogReader(wpilog.toString());
        if (!reader.isValid()) {
            throw new IllegalStateException("Invalid WPILOG file: " + wpilog);
        }

        Map<Integer, EntryInfo> entries = scanEntries(wpilog);
        EntryIds entryIds = EntryIds.resolve(entries);
        Timeline timeline = buildTimeline(wpilog, entryIds);

        List<Sample> samples = sanitizeSamples(timeline.samples);

        List<CommandSession> aimOnlySessions = buildCommandSessions(
                samples,
                timeline.lifecycleEvents,
                "ShooterDriverAim",
                Sample::shooterDriverAimRunning,
                Sample::shooterDriverAimActiveInstances);
        List<CommandSession> shootSessions = buildCommandSessions(
                samples,
                timeline.lifecycleEvents,
                "ShooterSelectedShootMode",
                Sample::shooterSelectedShootModeRunning,
                Sample::shooterSelectedShootModeActiveInstances);
        List<CommandSession> pathOverrideSessions = buildSyntheticSessions(samples, WindowKind.AUTO_PATH, "AutoAimPathRotationOverride");
        List<WindowSummary> windows = buildWindows(samples);

        return new LogSummary(
                wpilog,
                matchNumber(wpilog),
                entryIds,
                timeline.parseWarning,
                samples,
                timeline.lifecycleEvents,
                aimOnlySessions,
                shootSessions,
                pathOverrideSessions,
                windows);
    }

    private static List<Sample> sanitizeSamples(List<Sample> samples) {
        List<Sample> sanitized = new ArrayList<>(samples.size());
        long lastTimestampUs = Long.MIN_VALUE;
        for (Sample sample : samples) {
            if (sample.timestampUs < 0) {
                continue;
            }
            if (sample.timestampUs > MAX_PLAUSIBLE_TIMESTAMP_US) {
                continue;
            }
            if (lastTimestampUs != Long.MIN_VALUE && sample.timestampUs < lastTimestampUs) {
                continue;
            }
            sanitized.add(sample);
            lastTimestampUs = sample.timestampUs;
        }
        return sanitized;
    }

    private static Map<Integer, EntryInfo> scanEntries(Path wpilog) throws IOException {
        DataLogReader reader = new DataLogReader(wpilog.toString());
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
            } catch (RuntimeException ignored) {
                break;
            }
        }
        return entries;
    }

    private static Timeline buildTimeline(Path wpilog, EntryIds ids) throws IOException {
        List<Sample> samples = new ArrayList<>();
        List<LifecycleEvent> lifecycleEvents = new ArrayList<>();
        StringBuilder parseWarning = new StringBuilder();

        DataLogReader reader = new DataLogReader(wpilog.toString());
        Iterator<DataLogRecord> iterator = reader.iterator();
        State state = new State();
        long activeTimestampUs = Long.MIN_VALUE;
        boolean sawAnyRecordAtTimestamp = false;

        while (true) {
            final DataLogRecord record;
            try {
                if (!iterator.hasNext()) {
                    break;
                }
                record = iterator.next();
            } catch (RuntimeException exception) {
                if (parseWarning.length() > 0) {
                    parseWarning.append("; ");
                }
                parseWarning.append(exception.getClass().getSimpleName()).append(": ").append(exception.getMessage());
                break;
            }

            if (record.isStart() || record.isControl()) {
                continue;
            }

            long timestampUs = record.getTimestamp();
            if (activeTimestampUs == Long.MIN_VALUE) {
                activeTimestampUs = timestampUs;
            } else if (timestampUs != activeTimestampUs) {
                if (sawAnyRecordAtTimestamp) {
                    samples.add(state.snapshot(activeTimestampUs));
                }
                activeTimestampUs = timestampUs;
                sawAnyRecordAtTimestamp = false;
            }
            sawAnyRecordAtTimestamp = true;

            int entry = record.getEntry();
            if (entry == ids.modeEntry) {
                state.mode = record.getString();
            } else if (entry == ids.enabledEntry) {
                state.enabled = readBooleanLenient(record, ids.modeNameFor(entry));
            } else if (entry == ids.matchTimeEntry) {
                state.matchTime = readDoubleLenient(record);
            } else if (entry == ids.autoAimOverrideEnabledEntry) {
                state.autoAimOverrideEnabled = readBooleanLenient(record, ids.modeNameFor(entry));
            } else if (entry == ids.aimTargetAvailableEntry) {
                state.aimTargetAvailable = readBooleanLenient(record, ids.modeNameFor(entry));
            } else if (entry == ids.aimReadyEntry) {
                state.aimReady = readBooleanLenient(record, ids.modeNameFor(entry));
                state.aimReadyTsUs = timestampUs;
            } else if (entry == ids.aimReadyLatchedEntry) {
                state.aimReadyLatched = readBooleanLenient(record, ids.modeNameFor(entry));
                state.aimReadyLatchedTsUs = timestampUs;
            } else if (entry == ids.aimToleranceRadEntry) {
                state.aimToleranceRad = readDoubleLenient(record);
            } else if (entry == ids.aimReleaseToleranceRadEntry) {
                state.aimReleaseToleranceRad = readDoubleLenient(record);
            } else if (entry == ids.shootingDesiredHeadingDegEntry) {
                state.shootingDesiredHeadingDeg = readDoubleLenient(record);
                state.shootingDesiredHeadingDegTsUs = timestampUs;
            } else if (entry == ids.shooterDesiredHeadingDegEntry) {
                state.shooterDesiredHeadingDeg = readDoubleLenient(record);
                state.shooterDesiredHeadingDegTsUs = timestampUs;
            } else if (entry == ids.shooterDesiredHeadingRateRadPerSecEntry) {
                state.shooterDesiredHeadingRateRadPerSec = readDoubleLenient(record);
                state.shooterDesiredHeadingRateTsUs = timestampUs;
            } else if (entry == ids.shootingAimErrorDegEntry) {
                state.shootingAimErrorDeg = readDoubleLenient(record);
                state.shootingAimErrorDegTsUs = timestampUs;
            } else if (entry == ids.shotYawHeadingErrorDegEntry) {
                state.shotYawHeadingErrorDeg = readDoubleLenient(record);
                state.shotYawHeadingErrorDegTsUs = timestampUs;
            } else if (entry == ids.shotYawDesiredHeadingDegEntry) {
                state.shotYawDesiredHeadingDeg = readDoubleLenient(record);
                state.shotYawDesiredHeadingDegTsUs = timestampUs;
            } else if (entry == ids.shotYawDesiredHeadingRateRadPerSecEntry) {
                state.shotYawDesiredHeadingRateRadPerSec = readDoubleLenient(record);
                state.shotYawDesiredHeadingRateTsUs = timestampUs;
            } else if (entry == ids.shotYawFeedbackOmegaRadPerSecEntry) {
                state.shotYawFeedbackOmegaRadPerSec = readDoubleLenient(record);
                state.shotYawFeedbackOmegaTsUs = timestampUs;
            } else if (entry == ids.shotYawMeasuredOmegaRadPerSecEntry) {
                state.shotYawMeasuredOmegaRadPerSec = readDoubleLenient(record);
                state.shotYawMeasuredOmegaTsUs = timestampUs;
            } else if (entry == ids.shotYawOmegaCommandRadPerSecEntry) {
                state.shotYawOmegaCommandRadPerSec = readDoubleLenient(record);
                state.shotYawOmegaCommandTsUs = timestampUs;
            } else if (entry == ids.pathRotationOverrideEnabledEntry) {
                state.pathRotationOverrideEnabled = readBooleanLenient(record, ids.modeNameFor(entry));
            } else if (entry == ids.pathRotationOverrideTargetDegEntry) {
                state.pathRotationOverrideTargetDeg = readDoubleLenient(record);
                state.pathRotationOverrideTargetTsUs = timestampUs;
            } else if (entry == ids.pathRotationOverrideTargetRateDegPerSecEntry) {
                state.pathRotationOverrideTargetRateDegPerSec = readDoubleLenient(record);
                state.pathRotationOverrideTargetRateTsUs = timestampUs;
            } else if (entry == ids.requestedSpeedsEntry) {
                state.requestedSpeeds = readChassisSample(record, ids.typeFor(entry));
                state.requestedSpeedsTsUs = timestampUs;
            } else if (entry == ids.setpointSpeedsEntry) {
                state.setpointSpeeds = readChassisSample(record, ids.typeFor(entry));
                state.setpointSpeedsTsUs = timestampUs;
            } else if (entry == ids.measuredSpeedsEntry) {
                state.measuredSpeeds = readChassisSample(record, ids.typeFor(entry));
                state.measuredSpeedsTsUs = timestampUs;
            } else if (entry == ids.shooterDriverAimRunningEntry) {
                state.shooterDriverAimRunning = readBooleanLenient(record, ids.modeNameFor(entry));
            } else if (entry == ids.shooterSelectedShootModeRunningEntry) {
                state.shooterSelectedShootModeRunning = readBooleanLenient(record, ids.modeNameFor(entry));
            } else if (entry == ids.shooterDriverAimActiveInstancesEntry) {
                state.shooterDriverAimActiveInstances = readLongLenient(record);
            } else if (entry == ids.shooterSelectedShootModeActiveInstancesEntry) {
                state.shooterSelectedShootModeActiveInstances = readLongLenient(record);
            } else if (entry == ids.lastStartedNameEntry) {
                state.lastStartedName = record.getString();
            } else if (entry == ids.lastStartedSourceEntry) {
                state.lastStartedSource = record.getString();
            } else if (entry == ids.lastStartedRunIdEntry) {
                lifecycleEvents.add(new LifecycleEvent(
                        timestampUs,
                        LifecycleKind.START,
                        emptyIfNull(state.lastStartedName),
                        (int) readLongLenient(record),
                        emptyIfNull(state.lastStartedSource),
                        false,
                        Double.NaN));
            } else if (entry == ids.lastEndedNameEntry) {
                state.lastEndedName = record.getString();
            } else if (entry == ids.lastEndedSourceEntry) {
                state.lastEndedSource = record.getString();
            } else if (entry == ids.lastEndedInterruptedEntry) {
                state.lastEndedInterrupted = readBooleanLenient(record, ids.modeNameFor(entry));
            } else if (entry == ids.lastEndedDurationSecEntry) {
                state.lastEndedDurationSec = readDoubleLenient(record);
            } else if (entry == ids.lastEndedRunIdEntry) {
                lifecycleEvents.add(new LifecycleEvent(
                        timestampUs,
                        state.lastEndedInterrupted ? LifecycleKind.INTERRUPT : LifecycleKind.FINISH,
                        emptyIfNull(state.lastEndedName),
                        (int) readLongLenient(record),
                        emptyIfNull(state.lastEndedSource),
                        state.lastEndedInterrupted,
                        state.lastEndedDurationSec));
            } else if (entry == ids.lastCancelAllSourceEntry) {
                lifecycleEvents.add(new LifecycleEvent(
                        timestampUs,
                        LifecycleKind.CANCEL_ALL,
                        "<cancelAll>",
                        -1,
                        record.getString(),
                        true,
                        Double.NaN));
            }
        }

        if (activeTimestampUs != Long.MIN_VALUE && sawAnyRecordAtTimestamp) {
            samples.add(state.snapshot(activeTimestampUs));
        }

        return new Timeline(samples, lifecycleEvents, parseWarning.toString());
    }

    private static List<CommandSession> buildCommandSessions(
            List<Sample> samples,
            List<LifecycleEvent> lifecycleEvents,
            String commandName,
            RunningSelector runningSelector,
            ActiveInstancesSelector activeInstancesSelector) {
        List<LifecycleEvent> starts = lifecycleEvents.stream()
                .filter(event -> event.kind == LifecycleKind.START && commandName.equals(event.name))
                .collect(Collectors.toCollection(ArrayList::new));
        List<LifecycleEvent> ends = lifecycleEvents.stream()
                .filter(event -> (event.kind == LifecycleKind.FINISH || event.kind == LifecycleKind.INTERRUPT)
                        && commandName.equals(event.name))
                .collect(Collectors.toCollection(ArrayList::new));
        boolean[] startUsed = new boolean[starts.size()];
        boolean[] endUsed = new boolean[ends.size()];

        List<CommandSession> sessions = new ArrayList<>();
        CommandSessionBuilder active = null;
        Sample previous = null;
        for (Sample sample : samples) {
            boolean running = runningSelector.get(sample);
            boolean wasRunning = previous != null && runningSelector.get(previous);
            if (running && !wasRunning) {
                LifecycleEvent start = takeNearest(starts, startUsed, sample.timestampUs, 80_000L);
                active = new CommandSessionBuilder(commandName, sample, start);
            }
            if (active != null && running) {
                active.accept(sample, activeInstancesSelector.get(sample));
            }
            if (!running && wasRunning && active != null) {
                LifecycleEvent end = takeNearest(ends, endUsed, sample.timestampUs, 120_000L);
                sessions.add(active.finish(sample, end));
                active = null;
            }
            previous = sample;
        }
        if (active != null && previous != null) {
            LifecycleEvent end = takeNearestAfter(ends, endUsed, previous.timestampUs, 250_000L);
            sessions.add(active.finish(previous, end));
        }
        return sessions;
    }

    private static List<CommandSession> buildSyntheticSessions(List<Sample> samples, WindowKind kind, String name) {
        List<CommandSession> sessions = new ArrayList<>();
        Sample start = null;
        Sample previous = null;
        long maxInstances = 1;
        for (Sample sample : samples) {
            boolean active = sample.windowKind() == kind;
            boolean wasActive = previous != null && previous.windowKind() == kind;
            if (active && !wasActive) {
                start = sample;
                maxInstances = 1;
            }
            if (active) {
                maxInstances = 1;
            }
            if (!active && wasActive && start != null && previous != null) {
                sessions.add(new CommandSession(
                        name,
                        start.timestampUs,
                        previous.timestampUs,
                        start.mode,
                        start.matchTime,
                        previous.matchTime,
                        maxInstances,
                        false,
                        Double.NaN,
                        "",
                        ""));
                start = null;
            }
            previous = sample;
        }
        if (start != null && previous != null) {
            sessions.add(new CommandSession(
                    name,
                    start.timestampUs,
                    previous.timestampUs,
                    start.mode,
                    start.matchTime,
                    previous.matchTime,
                    maxInstances,
                    false,
                    Double.NaN,
                    "",
                    ""));
        }
        return sessions;
    }

    private static LifecycleEvent takeNearest(List<LifecycleEvent> events, boolean[] used, long timestampUs, long toleranceUs) {
        for (int i = 0; i < events.size(); i++) {
            if (used[i]) {
                continue;
            }
            LifecycleEvent event = events.get(i);
            if (Math.abs(event.timestampUs - timestampUs) <= toleranceUs) {
                used[i] = true;
                return event;
            }
        }
        return null;
    }

    private static LifecycleEvent takeNearestAfter(List<LifecycleEvent> events, boolean[] used, long timestampUs, long toleranceUs) {
        for (int i = 0; i < events.size(); i++) {
            if (used[i]) {
                continue;
            }
            LifecycleEvent event = events.get(i);
            if (event.timestampUs >= timestampUs && event.timestampUs - timestampUs <= toleranceUs) {
                used[i] = true;
                return event;
            }
        }
        return null;
    }

    private static List<WindowSummary> buildWindows(List<Sample> samples) {
        List<WindowSummary> windows = new ArrayList<>();
        WindowBuilder active = null;
        Sample previous = null;
        for (Sample sample : samples) {
            WindowKind kind = sample.windowKind();
            if (kind == WindowKind.NONE) {
                if (active != null) {
                    windows.add(active.finish());
                    active = null;
                }
                previous = sample;
                continue;
            }
            boolean breakForGap = previous != null && sample.timestampUs - previous.timestampUs > WINDOW_BREAK_US;
            if (active == null || active.kind != kind || breakForGap) {
                if (active != null) {
                    windows.add(active.finish());
                }
                active = new WindowBuilder(kind, sample);
            }
            active.accept(sample);
            previous = sample;
        }
        if (active != null) {
            windows.add(active.finish());
        }
        return windows;
    }

    private static String formatReport(Path logDir, List<LogSummary> summaries) {
        StringBuilder out = new StringBuilder();
        out.append("Auto aim heading analysis\n");
        out.append("logDir=").append(logDir).append('\n');
        out.append("matches=").append(summaries.stream().map(summary -> Integer.toString(summary.matchNumber)).collect(Collectors.joining(", "))).append("\n\n");

        Aggregate aggregate = Aggregate.from(summaries);
        out.append("overall summary\n");
        out.append(String.format(Locale.US, "  logs=%d samples=%d windows=%d teleopAimWindows=%d teleopShootWindows=%d autoPathWindows=%d\n",
                summaries.size(),
                aggregate.totalSamples,
                aggregate.totalWindows,
                aggregate.windowCounts.get(WindowKind.TELEOP_AIM_ONLY),
                aggregate.windowCounts.get(WindowKind.TELEOP_SHOOT),
                aggregate.windowCounts.get(WindowKind.AUTO_PATH)));
        out.append(String.format(Locale.US, "  teleopAimSessions=%d shootSessions=%d autoPathSessions=%d totalHeadingActive=%.2fs\n",
                aggregate.totalAimOnlySessions,
                aggregate.totalShootSessions,
                aggregate.totalAutoPathSessions,
                aggregate.totalWindowDurationSec));
        out.append(String.format(Locale.US, "  maxWindowGap=%.3fs windowsWithGap>%.0fms=%d gap>%.0fms=%d\n",
                aggregate.maxGapSec,
                GAP_WARN_SEC * 1000.0,
                aggregate.windowsWithGapWarn,
                GAP_BAD_SEC * 1000.0,
                aggregate.windowsWithGapBad));
        out.append(String.format(Locale.US, "  commandSessions interrupted=%d short<%.0fms=%d duplicateActiveInstances=%d cancelAllDuringHeading=%d\n",
                aggregate.interruptedCommandSessions,
                SHORT_RUN_WARN_SEC * 1000.0,
                aggregate.shortCommandSessions,
                aggregate.duplicateActiveInstancesSessions,
                aggregate.cancelAllDuringHeadingCount));
        out.append(String.format(Locale.US, "  absAimErrorDeg p50=%.2f p90=%.2f max=%.2f readyFraction=%.1f%% windowsReady=%d/%d\n",
                percentile(aggregate.absAimErrorsDeg, 0.50),
                percentile(aggregate.absAimErrorsDeg, 0.90),
                max(aggregate.absAimErrorsDeg),
                aggregate.readyFraction() * 100.0,
                aggregate.readyWindows,
                aggregate.totalWindows));
        out.append(String.format(Locale.US, "  signAgreement commandVsError=%.1f%% measuredVsError=%.1f%% requestVsTeleopCmd p90Diff=%.3frad/s\n",
                100.0 * fraction(aggregate.commandDirectionAgree, aggregate.commandDirectionTotal),
                100.0 * fraction(aggregate.measuredDirectionAgree, aggregate.measuredDirectionTotal),
                percentile(aggregate.teleopRequestedVsShotYawDiffRadPerSec, 0.90)));
        out.append(String.format(Locale.US, "  headingConsistency teleop(ShotYaw vs Shooting) max=%.2fdeg auto(Path vs Shooting) max=%.2fdeg shooterVsShooting max=%.2fdeg\n",
                max(aggregate.teleopHeadingConsistencyDeg),
                max(aggregate.autoHeadingConsistencyDeg),
                max(aggregate.shooterHeadingConsistencyDeg)));
        out.append(String.format(Locale.US, "  aimReady violations=%d zeroCommandAnomalySamples=%d desiredHeadingFreshMissSamples=%d\n\n",
                aggregate.aimReadyViolationSamples,
                aggregate.zeroCommandSamples,
                aggregate.desiredHeadingMissingSamples));

        out.append("per-match summary\n");
        for (LogSummary summary : summaries) {
            appendMatchSummary(out, summary);
        }

        List<String> findings = aggregate.suspiciousFindings();
        out.append("global findings\n");
        if (findings.isEmpty()) {
            out.append("  none\n");
        } else {
            for (String finding : findings) {
                out.append("  ").append(finding).append('\n');
            }
        }
        out.append('\n');

        out.append("worst windows by gap\n");
        appendTopWindows(out, summaries, Comparator.comparingDouble((WindowRef ref) -> ref.window.maxGapSec).reversed(), 8);
        out.append("worst windows by abs aim error\n");
        appendTopWindows(out, summaries, Comparator.comparingDouble((WindowRef ref) -> ref.window.maxAbsAimErrorDeg).reversed(), 8);
        out.append("slowest windows to first ready\n");
        appendTopWindows(out, summaries, Comparator.comparingDouble((WindowRef ref) -> finiteOrNegative(ref.window.timeToFirstReadySec)).reversed(), 8);

        out.append("command lifecycle detail\n");
        for (LogSummary summary : summaries) {
            appendCommandDetail(out, summary, "ShooterDriverAim", summary.aimOnlySessions);
            appendCommandDetail(out, summary, "ShooterSelectedShootMode", summary.shootSessions);
            appendCommandDetail(out, summary, "AutoAimPathRotationOverride", summary.pathOverrideSessions);
        }

        return out.toString();
    }

    private static void appendMatchSummary(StringBuilder out, LogSummary summary) {
        Aggregate matchAgg = Aggregate.from(List.of(summary));
        out.append(String.format(Locale.US, "  q%d %s\n", summary.matchNumber, summary.wpilog.getFileName()));
        out.append(String.format(Locale.US,
                "    windows total=%d teleopAim=%d teleopShoot=%d autoPath=%d headingActive=%.2fs\n",
                summary.windows.size(),
                summary.windowCounts().get(WindowKind.TELEOP_AIM_ONLY),
                summary.windowCounts().get(WindowKind.TELEOP_SHOOT),
                summary.windowCounts().get(WindowKind.AUTO_PATH),
                summary.totalWindowDurationSec()));
        out.append(String.format(Locale.US,
                "    commandSessions aimOnly=%d shoot=%d interrupted=%d short=%d duplicateInstances=%d cancelAllDuringHeading=%d\n",
                summary.aimOnlySessions.size(),
                summary.shootSessions.size(),
                matchAgg.interruptedCommandSessions,
                matchAgg.shortCommandSessions,
                matchAgg.duplicateActiveInstancesSessions,
                matchAgg.cancelAllDuringHeadingCount));
        out.append(String.format(Locale.US,
                "    absAimErrorDeg p50=%.2f p90=%.2f max=%.2f readyFraction=%.1f%% maxGap=%.3fs\n",
                percentile(summary.collectAbsAimErrors(), 0.50),
                percentile(summary.collectAbsAimErrors(), 0.90),
                max(summary.collectAbsAimErrors()),
                100.0 * summary.readyFraction(),
                summary.maxGapSec()));
        out.append(String.format(Locale.US,
                "    signAgreement command=%.1f%% measured=%.1f%% requestVsCmd p90Diff=%.3f headingDiff teleopMax=%.2f autoMax=%.2f shooterMax=%.2f\n",
                100.0 * fraction(summary.commandDirectionAgree(), summary.commandDirectionTotal()),
                100.0 * fraction(summary.measuredDirectionAgree(), summary.measuredDirectionTotal()),
                percentile(summary.collectTeleopRequestedVsShotYawDiff(), 0.90),
                max(summary.collectTeleopHeadingConsistency()),
                max(summary.collectAutoHeadingConsistency()),
                max(summary.collectShooterHeadingConsistency())));

        List<String> findings = summary.suspiciousFindings();
        if (findings.isEmpty()) {
            out.append("    findings: none\n");
        } else {
            out.append("    findings:\n");
            for (String finding : findings) {
                out.append("      - ").append(finding).append('\n');
            }
        }
        if (!summary.parseWarning.isBlank()) {
            out.append("    parseWarning: ").append(summary.parseWarning).append('\n');
        }
    }

    private static void appendTopWindows(
            StringBuilder out,
            List<LogSummary> summaries,
            Comparator<WindowRef> comparator,
            int limit) {
        List<WindowRef> refs = summaries.stream()
                .flatMap(summary -> summary.windows.stream().map(window -> new WindowRef(summary.matchNumber, summary.wpilog, window)))
                .sorted(comparator)
                .limit(limit)
                .collect(Collectors.toList());
        if (refs.isEmpty()) {
            out.append("  <none>\n\n");
            return;
        }
        for (WindowRef ref : refs) {
            WindowSummary w = ref.window;
            out.append(String.format(Locale.US,
                    "  q%d %-11s t=%.3f..%.3f dur=%.2fs maxGap=%.3fs p90Err=%.2f maxErr=%.2f ready=%.1f%% firstReady=%s zeroCmd=%d desiredMiss=%d issues=%s\n",
                    ref.matchNumber,
                    w.kind,
                    w.startTsUs / 1_000_000.0,
                    w.endTsUs / 1_000_000.0,
                    w.durationSec(),
                    w.maxGapSec,
                    w.p90AbsAimErrorDeg,
                    w.maxAbsAimErrorDeg,
                    w.readyFraction * 100.0,
                    formatMaybeNumber(w.timeToFirstReadySec, "%.2fs"),
                    w.zeroCommandAnomalySamples,
                    w.desiredHeadingMissingSamples,
                    w.issues.isEmpty() ? "none" : String.join(", ", w.issues)));
        }
        out.append('\n');
    }

    private static void appendCommandDetail(StringBuilder out, LogSummary summary, String name, List<CommandSession> sessions) {
        out.append(String.format(Locale.US, "  q%d %s sessions=%d\n", summary.matchNumber, name, sessions.size()));
        if (sessions.isEmpty()) {
            out.append("    <none>\n");
            return;
        }
        for (CommandSession session : sessions) {
            out.append(String.format(Locale.US,
                    "    t=%.3f..%.3f dur=%.2fs mode=%s activeInstancesMax=%d interrupted=%s startSource=%s endSource=%s\n",
                    session.startTsUs / 1_000_000.0,
                    session.endTsUs / 1_000_000.0,
                    session.durationSec(),
                    session.mode,
                    session.maxActiveInstances,
                    session.interrupted,
                    emptyIfNull(session.startSource),
                    emptyIfNull(session.endSource)));
        }
    }

    private static double finiteOrNegative(double value) {
        return Double.isFinite(value) ? value : -1.0;
    }

    private static String formatMaybeNumber(double value, String format) {
        return Double.isFinite(value) ? String.format(Locale.US, format, value) : "<never>";
    }

    private static String emptyIfNull(String value) {
        return value == null ? "" : value;
    }

    private static double readDoubleLenient(DataLogRecord record) {
        try {
            return record.getDouble();
        } catch (Exception ignored) {
        }
        try {
            return (double) record.getInteger();
        } catch (Exception ignored) {
        }
        try {
            return Double.parseDouble(record.getString());
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

    private static ChassisSample readChassisSample(DataLogRecord record, String type) {
        try {
            double[] values = record.getDoubleArray();
            if (values.length >= 3) {
                return new ChassisSample(values[0], values[1], values[2]);
            }
        } catch (Exception ignored) {
        }
        try {
            String text = record.getString();
            List<Double> values = new ArrayList<>();
            int idx = 0;
            while (idx < text.length()) {
                int start = idx;
                while (start < text.length() && !(text.charAt(start) == '-' || Character.isDigit(text.charAt(start)))) {
                    start++;
                }
                if (start >= text.length()) {
                    break;
                }
                int end = start + 1;
                while (end < text.length() && (Character.isDigit(text.charAt(end))
                        || text.charAt(end) == '.'
                        || text.charAt(end) == 'E'
                        || text.charAt(end) == 'e'
                        || text.charAt(end) == '-')) {
                    end++;
                }
                try {
                    values.add(Double.parseDouble(text.substring(start, end)));
                } catch (NumberFormatException ignored) {
                }
                idx = end;
            }
            if (values.size() >= 3) {
                return new ChassisSample(values.get(0), values.get(1), values.get(2));
            }
        } catch (Exception ignored) {
        }
        return null;
    }

    private static double fraction(long numerator, long denominator) {
        return denominator <= 0 ? Double.NaN : ((double) numerator) / denominator;
    }

    private static double percentile(List<Double> values, double percentile) {
        if (values.isEmpty()) {
            return Double.NaN;
        }
        List<Double> finite = values.stream().filter(Double::isFinite).sorted().collect(Collectors.toList());
        if (finite.isEmpty()) {
            return Double.NaN;
        }
        int index = (int) Math.round(percentile * (finite.size() - 1));
        index = Math.max(0, Math.min(index, finite.size() - 1));
        return finite.get(index);
    }

    private static double max(List<Double> values) {
        return values.stream().filter(Double::isFinite).mapToDouble(Double::doubleValue).max().orElse(Double.NaN);
    }

    private record EntryInfo(String name, String type) {}

    private record ChassisSample(double vx, double vy, double omega) {}

    private enum LifecycleKind {
        START,
        FINISH,
        INTERRUPT,
        CANCEL_ALL
    }

    private record LifecycleEvent(
            long timestampUs,
            LifecycleKind kind,
            String name,
            int runId,
            String source,
            boolean interrupted,
            double durationSec) {}

    private enum WindowKind {
        NONE,
        TELEOP_AIM_ONLY,
        TELEOP_SHOOT,
        AUTO_PATH
    }

    private interface RunningSelector {
        boolean get(Sample sample);
    }

    private interface ActiveInstancesSelector {
        long get(Sample sample);
    }

    private static final class EntryIds {
        private final Map<Integer, EntryInfo> entries;
        private int modeEntry = -1;
        private int enabledEntry = -1;
        private int matchTimeEntry = -1;
        private int autoAimOverrideEnabledEntry = -1;
        private int aimTargetAvailableEntry = -1;
        private int aimReadyEntry = -1;
        private int aimReadyLatchedEntry = -1;
        private int aimToleranceRadEntry = -1;
        private int aimReleaseToleranceRadEntry = -1;
        private int shootingDesiredHeadingDegEntry = -1;
        private int shooterDesiredHeadingDegEntry = -1;
        private int shooterDesiredHeadingRateRadPerSecEntry = -1;
        private int shootingAimErrorDegEntry = -1;
        private int shotYawHeadingErrorDegEntry = -1;
        private int shotYawDesiredHeadingDegEntry = -1;
        private int shotYawDesiredHeadingRateRadPerSecEntry = -1;
        private int shotYawFeedbackOmegaRadPerSecEntry = -1;
        private int shotYawMeasuredOmegaRadPerSecEntry = -1;
        private int shotYawOmegaCommandRadPerSecEntry = -1;
        private int pathRotationOverrideEnabledEntry = -1;
        private int pathRotationOverrideTargetDegEntry = -1;
        private int pathRotationOverrideTargetRateDegPerSecEntry = -1;
        private int requestedSpeedsEntry = -1;
        private int setpointSpeedsEntry = -1;
        private int measuredSpeedsEntry = -1;
        private int shooterDriverAimRunningEntry = -1;
        private int shooterSelectedShootModeRunningEntry = -1;
        private int shooterDriverAimActiveInstancesEntry = -1;
        private int shooterSelectedShootModeActiveInstancesEntry = -1;
        private int lastStartedNameEntry = -1;
        private int lastStartedSourceEntry = -1;
        private int lastStartedRunIdEntry = -1;
        private int lastEndedNameEntry = -1;
        private int lastEndedSourceEntry = -1;
        private int lastEndedInterruptedEntry = -1;
        private int lastEndedDurationSecEntry = -1;
        private int lastEndedRunIdEntry = -1;
        private int lastCancelAllSourceEntry = -1;

        private EntryIds(Map<Integer, EntryInfo> entries) {
            this.entries = entries;
        }

        static EntryIds resolve(Map<Integer, EntryInfo> entries) {
            EntryIds ids = new EntryIds(entries);
            ids.modeEntry = findEntry(entries, "RobotState/Mode");
            ids.enabledEntry = findEntry(entries, "RobotState/Enabled");
            ids.matchTimeEntry = findEntry(entries, "RobotState/MatchTime");
            ids.autoAimOverrideEnabledEntry = findEntry(entries, "Dashboard/Overrides/AutoAimEnabled");
            ids.aimTargetAvailableEntry = findEntry(entries, "Shooting/AimTargetAvailable");
            ids.aimReadyEntry = findEntry(entries, "Shooting/AimReady");
            ids.aimReadyLatchedEntry = findEntry(entries, "Shooting/AimReadyLatched");
            ids.aimToleranceRadEntry = findEntry(entries, "Shooting/ActiveAimToleranceRad", "Shooting/AimToleranceRad");
            ids.aimReleaseToleranceRadEntry = findEntry(entries, "Shooting/ActiveAimReleaseToleranceRad");
            ids.shootingDesiredHeadingDegEntry = findEntry(entries, "Shooting/DesiredRobotHeadingDeg");
            ids.shooterDesiredHeadingDegEntry = findEntry(entries, "Shooter/DesiredRobotHeadingDeg");
            ids.shooterDesiredHeadingRateRadPerSecEntry = findEntry(entries, "Shooter/DesiredRobotHeadingRateRadPerSec");
            ids.shootingAimErrorDegEntry = findEntry(entries, "Shooting/AimErrorDeg");
            ids.shotYawHeadingErrorDegEntry = findEntry(entries, "ShotYaw/HeadingErrorDeg");
            ids.shotYawDesiredHeadingDegEntry = findEntry(entries, "ShotYaw/DesiredHeadingDeg");
            ids.shotYawDesiredHeadingRateRadPerSecEntry = findEntry(entries, "ShotYaw/DesiredHeadingRateRadPerSec");
            ids.shotYawFeedbackOmegaRadPerSecEntry = findEntry(entries, "ShotYaw/FeedbackOmegaRadPerSec");
            ids.shotYawMeasuredOmegaRadPerSecEntry = findEntry(entries, "ShotYaw/MeasuredOmegaRadPerSec");
            ids.shotYawOmegaCommandRadPerSecEntry = findEntry(entries, "ShotYaw/OmegaCommandRadPerSec");
            ids.pathRotationOverrideEnabledEntry = findEntry(entries, "AutoAim/PathRotationOverrideEnabled");
            ids.pathRotationOverrideTargetDegEntry = findEntry(entries, "AutoAim/PathRotationOverrideTargetDeg");
            ids.pathRotationOverrideTargetRateDegPerSecEntry = findEntry(entries, "AutoAim/PathRotationOverrideTargetRateDegPerSec");
            ids.requestedSpeedsEntry = findEntry(entries, "SwerveChassisSpeeds/Requested");
            ids.setpointSpeedsEntry = findEntry(entries, "SwerveChassisSpeeds/Setpoints");
            ids.measuredSpeedsEntry = findEntry(entries, "SwerveChassisSpeeds/Measured");
            ids.shooterDriverAimRunningEntry = findEntry(entries, "Commands/byName/ShooterDriverAim/running");
            ids.shooterSelectedShootModeRunningEntry = findEntry(entries, "Commands/byName/ShooterSelectedShootMode/running");
            ids.shooterDriverAimActiveInstancesEntry = findEntry(entries, "Commands/byName/ShooterDriverAim/activeInstances");
            ids.shooterSelectedShootModeActiveInstancesEntry = findEntry(entries, "Commands/byName/ShooterSelectedShootMode/activeInstances");
            ids.lastStartedNameEntry = findEntry(entries, "Commands/lastStarted/name");
            ids.lastStartedSourceEntry = findEntry(entries, "Commands/lastStarted/source");
            ids.lastStartedRunIdEntry = findEntry(entries, "Commands/lastStarted/runId");
            ids.lastEndedNameEntry = findEntry(entries, "Commands/lastEnded/name");
            ids.lastEndedSourceEntry = findEntry(entries, "Commands/lastEnded/source");
            ids.lastEndedInterruptedEntry = findEntry(entries, "Commands/lastEnded/interrupted");
            ids.lastEndedDurationSecEntry = findEntry(entries, "Commands/lastEnded/durationSec");
            ids.lastEndedRunIdEntry = findEntry(entries, "Commands/lastEnded/runId");
            ids.lastCancelAllSourceEntry = findEntry(entries, "Commands/lastCancelAllSource");
            return ids;
        }

        String typeFor(int entry) {
            EntryInfo info = entries.get(entry);
            return info != null ? info.type : "";
        }

        String modeNameFor(int entry) {
            EntryInfo info = entries.get(entry);
            return info != null ? info.name : "entry=" + entry;
        }

        private static int findEntry(Map<Integer, EntryInfo> entries, String... suffixes) {
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
        String mode = "";
        boolean enabled = false;
        double matchTime = Double.NaN;
        boolean autoAimOverrideEnabled = false;
        boolean aimTargetAvailable = false;
        long aimTargetAvailableTsUs = Long.MIN_VALUE;
        boolean aimReady = false;
        long aimReadyTsUs = Long.MIN_VALUE;
        boolean aimReadyLatched = false;
        long aimReadyLatchedTsUs = Long.MIN_VALUE;
        double aimToleranceRad = Double.NaN;
        double aimReleaseToleranceRad = Double.NaN;
        double shootingDesiredHeadingDeg = Double.NaN;
        long shootingDesiredHeadingDegTsUs = Long.MIN_VALUE;
        double shooterDesiredHeadingDeg = Double.NaN;
        long shooterDesiredHeadingDegTsUs = Long.MIN_VALUE;
        double shooterDesiredHeadingRateRadPerSec = Double.NaN;
        long shooterDesiredHeadingRateTsUs = Long.MIN_VALUE;
        double shootingAimErrorDeg = Double.NaN;
        long shootingAimErrorDegTsUs = Long.MIN_VALUE;
        double shotYawHeadingErrorDeg = Double.NaN;
        long shotYawHeadingErrorDegTsUs = Long.MIN_VALUE;
        double shotYawDesiredHeadingDeg = Double.NaN;
        long shotYawDesiredHeadingDegTsUs = Long.MIN_VALUE;
        double shotYawDesiredHeadingRateRadPerSec = Double.NaN;
        long shotYawDesiredHeadingRateTsUs = Long.MIN_VALUE;
        double shotYawFeedbackOmegaRadPerSec = Double.NaN;
        long shotYawFeedbackOmegaTsUs = Long.MIN_VALUE;
        double shotYawMeasuredOmegaRadPerSec = Double.NaN;
        long shotYawMeasuredOmegaTsUs = Long.MIN_VALUE;
        double shotYawOmegaCommandRadPerSec = Double.NaN;
        long shotYawOmegaCommandTsUs = Long.MIN_VALUE;
        boolean pathRotationOverrideEnabled = false;
        double pathRotationOverrideTargetDeg = Double.NaN;
        long pathRotationOverrideTargetTsUs = Long.MIN_VALUE;
        double pathRotationOverrideTargetRateDegPerSec = Double.NaN;
        long pathRotationOverrideTargetRateTsUs = Long.MIN_VALUE;
        ChassisSample requestedSpeeds = null;
        long requestedSpeedsTsUs = Long.MIN_VALUE;
        ChassisSample setpointSpeeds = null;
        long setpointSpeedsTsUs = Long.MIN_VALUE;
        ChassisSample measuredSpeeds = null;
        long measuredSpeedsTsUs = Long.MIN_VALUE;
        boolean shooterDriverAimRunning = false;
        boolean shooterSelectedShootModeRunning = false;
        long shooterDriverAimActiveInstances = 0L;
        long shooterSelectedShootModeActiveInstances = 0L;
        String lastStartedName = "";
        String lastStartedSource = "";
        String lastEndedName = "";
        String lastEndedSource = "";
        boolean lastEndedInterrupted = false;
        double lastEndedDurationSec = Double.NaN;

        Sample snapshot(long timestampUs) {
            return new Sample(
                    timestampUs,
                    mode,
                    enabled,
                    matchTime,
                    autoAimOverrideEnabled,
                    aimTargetAvailable,
                    aimReady,
                    aimReadyLatched,
                    aimReadyTsUs,
                    aimReadyLatchedTsUs,
                    aimToleranceRad,
                    aimReleaseToleranceRad,
                    freshDouble(shootingDesiredHeadingDeg, shootingDesiredHeadingDegTsUs, timestampUs),
                    freshDouble(shooterDesiredHeadingDeg, shooterDesiredHeadingDegTsUs, timestampUs),
                    freshDouble(shooterDesiredHeadingRateRadPerSec, shooterDesiredHeadingRateTsUs, timestampUs),
                    freshDouble(shootingAimErrorDeg, shootingAimErrorDegTsUs, timestampUs),
                    freshDouble(shotYawHeadingErrorDeg, shotYawHeadingErrorDegTsUs, timestampUs),
                    freshDouble(shotYawDesiredHeadingDeg, shotYawDesiredHeadingDegTsUs, timestampUs),
                    freshDouble(shotYawDesiredHeadingRateRadPerSec, shotYawDesiredHeadingRateTsUs, timestampUs),
                    freshDouble(shotYawFeedbackOmegaRadPerSec, shotYawFeedbackOmegaTsUs, timestampUs),
                    freshDouble(shotYawMeasuredOmegaRadPerSec, shotYawMeasuredOmegaTsUs, timestampUs),
                    freshDouble(shotYawOmegaCommandRadPerSec, shotYawOmegaCommandTsUs, timestampUs),
                    pathRotationOverrideEnabled,
                    freshDouble(pathRotationOverrideTargetDeg, pathRotationOverrideTargetTsUs, timestampUs),
                    freshDouble(pathRotationOverrideTargetRateDegPerSec, pathRotationOverrideTargetRateTsUs, timestampUs),
                    freshChassis(requestedSpeeds, requestedSpeedsTsUs, timestampUs),
                    freshChassis(setpointSpeeds, setpointSpeedsTsUs, timestampUs),
                    freshChassis(measuredSpeeds, measuredSpeedsTsUs, timestampUs),
                    shooterDriverAimRunning,
                    shooterSelectedShootModeRunning,
                    shooterDriverAimActiveInstances,
                    shooterSelectedShootModeActiveInstances);
        }

        private static double freshDouble(double value, long lastUpdateUs, long timestampUs) {
            return lastUpdateUs != Long.MIN_VALUE && timestampUs - lastUpdateUs <= FRESH_US ? value : Double.NaN;
        }

        private static ChassisSample freshChassis(ChassisSample value, long lastUpdateUs, long timestampUs) {
            return lastUpdateUs != Long.MIN_VALUE && timestampUs - lastUpdateUs <= FRESH_US ? value : null;
        }
    }

    private record Sample(
            long timestampUs,
            String mode,
            boolean enabled,
            double matchTime,
            boolean autoAimOverrideEnabled,
            boolean aimTargetAvailable,
            boolean aimReady,
            boolean aimReadyLatched,
            long aimReadyTsUs,
            long aimReadyLatchedTsUs,
            double aimToleranceRad,
            double aimReleaseToleranceRad,
            double shootingDesiredHeadingDeg,
            double shooterDesiredHeadingDeg,
            double shooterDesiredHeadingRateRadPerSec,
            double shootingAimErrorDeg,
            double shotYawHeadingErrorDeg,
            double shotYawDesiredHeadingDeg,
            double shotYawDesiredHeadingRateRadPerSec,
            double shotYawFeedbackOmegaRadPerSec,
            double shotYawMeasuredOmegaRadPerSec,
            double shotYawOmegaCommandRadPerSec,
            boolean pathRotationOverrideEnabled,
            double pathRotationOverrideTargetDeg,
            double pathRotationOverrideTargetRateDegPerSec,
            ChassisSample requestedSpeeds,
            ChassisSample setpointSpeeds,
            ChassisSample measuredSpeeds,
            boolean shooterDriverAimRunning,
            boolean shooterSelectedShootModeRunning,
            long shooterDriverAimActiveInstances,
            long shooterSelectedShootModeActiveInstances) {
        WindowKind windowKind() {
            if (!enabled) {
                return WindowKind.NONE;
            }
            if (pathRotationOverrideEnabled && Double.isFinite(pathRotationOverrideTargetDeg)) {
                return WindowKind.AUTO_PATH;
            }
            if (autoAimOverrideEnabled) {
                return WindowKind.NONE;
            }
            if (Double.isFinite(shotYawDesiredHeadingDeg) && shooterSelectedShootModeRunning) {
                return WindowKind.TELEOP_SHOOT;
            }
            if (Double.isFinite(shotYawDesiredHeadingDeg) && shooterDriverAimRunning) {
                return WindowKind.TELEOP_AIM_ONLY;
            }
            return WindowKind.NONE;
        }

        double primaryRequestedOmega() {
            if (windowKind() == WindowKind.TELEOP_AIM_ONLY || windowKind() == WindowKind.TELEOP_SHOOT) {
                return Double.isFinite(shotYawOmegaCommandRadPerSec)
                        ? shotYawOmegaCommandRadPerSec
                        : requestedSpeeds != null ? requestedSpeeds.omega : Double.NaN;
            }
            if (windowKind() == WindowKind.AUTO_PATH) {
                if (setpointSpeeds != null) {
                    return setpointSpeeds.omega;
                }
                return requestedSpeeds != null ? requestedSpeeds.omega : Double.NaN;
            }
            return Double.NaN;
        }

        double measuredOmega() {
            if (windowKind() == WindowKind.TELEOP_AIM_ONLY || windowKind() == WindowKind.TELEOP_SHOOT) {
                if (Double.isFinite(shotYawMeasuredOmegaRadPerSec)) {
                    return shotYawMeasuredOmegaRadPerSec;
                }
            }
            return measuredSpeeds != null ? measuredSpeeds.omega : Double.NaN;
        }

        double desiredHeadingDeg() {
            if (windowKind() == WindowKind.AUTO_PATH) {
                if (Double.isFinite(pathRotationOverrideTargetDeg)) {
                    return pathRotationOverrideTargetDeg;
                }
            }
            if (Double.isFinite(shotYawDesiredHeadingDeg)) {
                return shotYawDesiredHeadingDeg;
            }
            if (Double.isFinite(shootingDesiredHeadingDeg)) {
                return shootingDesiredHeadingDeg;
            }
            return Double.NaN;
        }

        double aimErrorDeg() {
            if (Double.isFinite(shootingAimErrorDeg)) {
                return shootingAimErrorDeg;
            }
            if (Double.isFinite(shotYawHeadingErrorDeg)) {
                return shotYawHeadingErrorDeg;
            }
            return Double.NaN;
        }
    }

    private static final class CommandSessionBuilder {
        private final String name;
        private final long startTsUs;
        private final String mode;
        private final double startMatchTime;
        private final String startSource;
        private long endTsUs;
        private double endMatchTime;
        private long maxActiveInstances;

        private CommandSessionBuilder(String name, Sample start, LifecycleEvent lifecycleStart) {
            this.name = name;
            this.startTsUs = start.timestampUs;
            this.endTsUs = start.timestampUs;
            this.mode = start.mode;
            this.startMatchTime = start.matchTime;
            this.endMatchTime = start.matchTime;
            this.startSource = lifecycleStart != null ? lifecycleStart.source : "";
        }

        void accept(Sample sample, long activeInstances) {
            endTsUs = sample.timestampUs;
            endMatchTime = sample.matchTime;
            maxActiveInstances = Math.max(maxActiveInstances, activeInstances);
        }

        CommandSession finish(Sample endSample, LifecycleEvent lifecycleEnd) {
            endTsUs = Math.max(endTsUs, endSample.timestampUs);
            endMatchTime = endSample.matchTime;
            return new CommandSession(
                    name,
                    startTsUs,
                    endTsUs,
                    mode,
                    startMatchTime,
                    endMatchTime,
                    maxActiveInstances,
                    lifecycleEnd != null && lifecycleEnd.interrupted,
                    lifecycleEnd != null ? lifecycleEnd.durationSec : Double.NaN,
                    startSource,
                    lifecycleEnd != null ? lifecycleEnd.source : "");
        }
    }

    private record CommandSession(
            String name,
            long startTsUs,
            long endTsUs,
            String mode,
            double startMatchTime,
            double endMatchTime,
            long maxActiveInstances,
            boolean interrupted,
            double recordedDurationSec,
            String startSource,
            String endSource) {
        double durationSec() {
            return Math.max(0.0, (endTsUs - startTsUs) / 1_000_000.0);
        }
    }

    private static final class WindowBuilder {
        private final WindowKind kind;
        private final long startTsUs;
        private final String mode;
        private final double startMatchTime;
        private final List<Double> absAimErrorsDeg = new ArrayList<>();
        private final List<Double> teleopRequestedVsShotYawDiffRadPerSec = new ArrayList<>();
        private final List<Double> teleopHeadingConsistencyDeg = new ArrayList<>();
        private final List<Double> autoHeadingConsistencyDeg = new ArrayList<>();
        private final List<Double> shooterHeadingConsistencyDeg = new ArrayList<>();
        private final List<String> issues = new ArrayList<>();
        private long endTsUs;
        private double endMatchTime;
        private Sample previousSample;
        private long sampleCount = 0;
        private long readySamples = 0;
        private double maxGapSec = 0.0;
        private long gapWarnCount = 0;
        private long gapBadCount = 0;
        private long commandDirectionAgree = 0;
        private long commandDirectionTotal = 0;
        private long measuredDirectionAgree = 0;
        private long measuredDirectionTotal = 0;
        private long zeroCommandAnomalySamples = 0;
        private long desiredHeadingMissingSamples = 0;
        private long aimReadyViolationSamples = 0;
        private long largeErrorSamples = 0;
        private double timeToFirstReadySec = Double.NaN;
        private double maxAbsAimErrorDeg = Double.NaN;
        private boolean readyObservedThisWindow = false;

        private WindowBuilder(WindowKind kind, Sample first) {
            this.kind = kind;
            this.startTsUs = first.timestampUs;
            this.endTsUs = first.timestampUs;
            this.mode = first.mode;
            this.startMatchTime = first.matchTime;
            this.endMatchTime = first.matchTime;
        }

        void accept(Sample sample) {
            sampleCount++;
            endTsUs = sample.timestampUs;
            endMatchTime = sample.matchTime;
            if (previousSample != null) {
                double gapSec = (sample.timestampUs - previousSample.timestampUs) / 1_000_000.0;
                maxGapSec = Math.max(maxGapSec, gapSec);
                if (gapSec > GAP_WARN_SEC) {
                    gapWarnCount++;
                }
                if (gapSec > GAP_BAD_SEC) {
                    gapBadCount++;
                }
            }

            double absAimErrorDeg = Math.abs(sample.aimErrorDeg());
            if (Double.isFinite(absAimErrorDeg)) {
                absAimErrorsDeg.add(absAimErrorDeg);
                maxAbsAimErrorDeg = Double.isNaN(maxAbsAimErrorDeg) ? absAimErrorDeg : Math.max(maxAbsAimErrorDeg, absAimErrorDeg);
                if (absAimErrorDeg > LARGE_ERROR_DEG) {
                    largeErrorSamples++;
                }
            }

            if (sample.aimReadyTsUs >= startTsUs || sample.aimReadyLatchedTsUs >= startTsUs) {
                readyObservedThisWindow = true;
            }
            boolean ready = readyObservedThisWindow && (sample.aimReady || sample.aimReadyLatched);
            if (ready) {
                readySamples++;
                if (!Double.isFinite(timeToFirstReadySec)) {
                    timeToFirstReadySec = (sample.timestampUs - startTsUs) / 1_000_000.0;
                }
            }

            if (kind == WindowKind.TELEOP_AIM_ONLY || kind == WindowKind.TELEOP_SHOOT) {
                if (Double.isFinite(sample.shotYawDesiredHeadingDeg) && Double.isFinite(sample.shootingDesiredHeadingDeg)) {
                    teleopHeadingConsistencyDeg.add(Math.abs(angleDiffDeg(sample.shotYawDesiredHeadingDeg, sample.shootingDesiredHeadingDeg)));
                }
                if (Double.isFinite(sample.shooterDesiredHeadingDeg) && Double.isFinite(sample.shootingDesiredHeadingDeg)) {
                    shooterHeadingConsistencyDeg.add(Math.abs(angleDiffDeg(sample.shooterDesiredHeadingDeg, sample.shootingDesiredHeadingDeg)));
                }
                if (sample.requestedSpeeds != null && Double.isFinite(sample.shotYawOmegaCommandRadPerSec)) {
                    teleopRequestedVsShotYawDiffRadPerSec.add(Math.abs(sample.requestedSpeeds.omega - sample.shotYawOmegaCommandRadPerSec));
                }
            } else if (kind == WindowKind.AUTO_PATH) {
                if (Double.isFinite(sample.pathRotationOverrideTargetDeg) && Double.isFinite(sample.shootingDesiredHeadingDeg)) {
                    autoHeadingConsistencyDeg.add(Math.abs(angleDiffDeg(sample.pathRotationOverrideTargetDeg, sample.shootingDesiredHeadingDeg)));
                }
                if (Double.isFinite(sample.shooterDesiredHeadingDeg) && Double.isFinite(sample.shootingDesiredHeadingDeg)) {
                    shooterHeadingConsistencyDeg.add(Math.abs(angleDiffDeg(sample.shooterDesiredHeadingDeg, sample.shootingDesiredHeadingDeg)));
                }
            }

            double desiredHeadingDeg = sample.desiredHeadingDeg();
            if (!Double.isFinite(desiredHeadingDeg)) {
                desiredHeadingMissingSamples++;
            }

            double errorDeg = sample.aimErrorDeg();
            double commandOmega = sample.primaryRequestedOmega();
            double measuredOmega = sample.measuredOmega();
            if (Double.isFinite(errorDeg) && Math.abs(errorDeg) > LARGE_ERROR_DEG) {
                if (Double.isFinite(commandOmega) && Math.abs(commandOmega) > LARGE_OMEGA_RAD_PER_SEC) {
                    commandDirectionTotal++;
                    if (Math.signum(commandOmega) == Math.signum(errorDeg)) {
                        commandDirectionAgree++;
                    }
                }
                if (Double.isFinite(measuredOmega) && Math.abs(measuredOmega) > LARGE_OMEGA_RAD_PER_SEC) {
                    measuredDirectionTotal++;
                    if (Math.signum(measuredOmega) == Math.signum(errorDeg)) {
                        measuredDirectionAgree++;
                    }
                }
                double toleranceDeg = Double.isFinite(sample.aimToleranceRad) ? Math.toDegrees(sample.aimToleranceRad) : LARGE_ERROR_DEG;
                if (Double.isFinite(commandOmega) && Math.abs(commandOmega) < ZERO_COMMAND_RAD_PER_SEC && Math.abs(errorDeg) > Math.max(LARGE_ERROR_DEG, toleranceDeg)) {
                    zeroCommandAnomalySamples++;
                }
            }

            if (ready && Double.isFinite(errorDeg)) {
                double allowedDeg = Double.isFinite(sample.aimReleaseToleranceRad)
                        ? Math.toDegrees(sample.aimReleaseToleranceRad) + READY_ERROR_MARGIN_DEG
                        : LARGE_ERROR_DEG;
                if (Math.abs(errorDeg) > allowedDeg) {
                    aimReadyViolationSamples++;
                }
            }

            previousSample = sample;
        }

        WindowSummary finish() {
            if (maxGapSec > GAP_WARN_SEC) {
                issues.add(String.format(Locale.US, "gapMax=%.3fs", maxGapSec));
            }
            if (gapBadCount > 0) {
                issues.add("gap>100ms=" + gapBadCount);
            }
            if (zeroCommandAnomalySamples > 0) {
                issues.add("zeroCmd=" + zeroCommandAnomalySamples);
            }
            if (desiredHeadingMissingSamples > 0) {
                issues.add("desiredMissing=" + desiredHeadingMissingSamples);
            }
            if (aimReadyViolationSamples > 0) {
                issues.add("readyViolation=" + aimReadyViolationSamples);
            }
            double commandAgree = fraction(commandDirectionAgree, commandDirectionTotal);
            if (Double.isFinite(commandAgree) && commandAgree < 0.85) {
                issues.add(String.format(Locale.US, "cmdAgree=%.1f%%", 100.0 * commandAgree));
            }
            double measuredAgree = fraction(measuredDirectionAgree, measuredDirectionTotal);
            if (Double.isFinite(measuredAgree) && measuredAgree < 0.65) {
                issues.add(String.format(Locale.US, "measAgree=%.1f%%", 100.0 * measuredAgree));
            }
            double teleopHeadingMax = max(teleopHeadingConsistencyDeg);
            if (Double.isFinite(teleopHeadingMax) && teleopHeadingMax > 0.50) {
                issues.add(String.format(Locale.US, "teleopHeadingDiff=%.2fdeg", teleopHeadingMax));
            }
            double autoHeadingMax = max(autoHeadingConsistencyDeg);
            if (Double.isFinite(autoHeadingMax) && autoHeadingMax > 0.50) {
                issues.add(String.format(Locale.US, "autoHeadingDiff=%.2fdeg", autoHeadingMax));
            }
            double shooterHeadingMax = max(shooterHeadingConsistencyDeg);
            if (Double.isFinite(shooterHeadingMax) && shooterHeadingMax > 0.50) {
                issues.add(String.format(Locale.US, "shooterHeadingDiff=%.2fdeg", shooterHeadingMax));
            }
            double requestCmdP90 = percentile(teleopRequestedVsShotYawDiffRadPerSec, 0.90);
            if (Double.isFinite(requestCmdP90) && requestCmdP90 > 0.25) {
                issues.add(String.format(Locale.US, "requestVsCmdP90=%.3f", requestCmdP90));
            }
            return new WindowSummary(
                    kind,
                    startTsUs,
                    endTsUs,
                    mode,
                    startMatchTime,
                    endMatchTime,
                    sampleCount,
                    maxGapSec,
                    gapWarnCount,
                    gapBadCount,
                    readySamples,
                    sampleCount <= 0 ? Double.NaN : ((double) readySamples) / sampleCount,
                    timeToFirstReadySec,
                    percentile(absAimErrorsDeg, 0.50),
                    percentile(absAimErrorsDeg, 0.90),
                    maxAbsAimErrorDeg,
                    absAimErrorsDeg,
                    teleopRequestedVsShotYawDiffRadPerSec,
                    teleopHeadingConsistencyDeg,
                    autoHeadingConsistencyDeg,
                    shooterHeadingConsistencyDeg,
                    commandDirectionAgree,
                    commandDirectionTotal,
                    measuredDirectionAgree,
                    measuredDirectionTotal,
                    zeroCommandAnomalySamples,
                    desiredHeadingMissingSamples,
                    aimReadyViolationSamples,
                    largeErrorSamples,
                    List.copyOf(issues));
        }
    }

    private static double angleDiffDeg(double aDeg, double bDeg) {
        double diff = aDeg - bDeg;
        while (diff > 180.0) {
            diff -= 360.0;
        }
        while (diff < -180.0) {
            diff += 360.0;
        }
        return diff;
    }

    private record WindowSummary(
            WindowKind kind,
            long startTsUs,
            long endTsUs,
            String mode,
            double startMatchTime,
            double endMatchTime,
            long sampleCount,
            double maxGapSec,
            long gapWarnCount,
            long gapBadCount,
            long readySamples,
            double readyFraction,
            double timeToFirstReadySec,
            double p50AbsAimErrorDeg,
            double p90AbsAimErrorDeg,
            double maxAbsAimErrorDeg,
            List<Double> absAimErrorsDeg,
            List<Double> teleopRequestedVsShotYawDiffRadPerSec,
            List<Double> teleopHeadingConsistencyDeg,
            List<Double> autoHeadingConsistencyDeg,
            List<Double> shooterHeadingConsistencyDeg,
            long commandDirectionAgree,
            long commandDirectionTotal,
            long measuredDirectionAgree,
            long measuredDirectionTotal,
            long zeroCommandAnomalySamples,
            long desiredHeadingMissingSamples,
            long aimReadyViolationSamples,
            long largeErrorSamples,
            List<String> issues) {
        double durationSec() {
            return Math.max(0.0, (endTsUs - startTsUs) / 1_000_000.0);
        }
    }

    private record Timeline(List<Sample> samples, List<LifecycleEvent> lifecycleEvents, String parseWarning) {}

    private record LogSummary(
            Path wpilog,
            int matchNumber,
            EntryIds entryIds,
            String parseWarning,
            List<Sample> samples,
            List<LifecycleEvent> lifecycleEvents,
            List<CommandSession> aimOnlySessions,
            List<CommandSession> shootSessions,
            List<CommandSession> pathOverrideSessions,
            List<WindowSummary> windows) {
        EnumMap<WindowKind, Integer> windowCounts() {
            EnumMap<WindowKind, Integer> counts = new EnumMap<>(WindowKind.class);
            for (WindowKind kind : WindowKind.values()) {
                counts.put(kind, 0);
            }
            for (WindowSummary window : windows) {
                counts.put(window.kind, counts.get(window.kind) + 1);
            }
            return counts;
        }

        double totalWindowDurationSec() {
            return windows.stream().mapToDouble(WindowSummary::durationSec).sum();
        }

        double readyFraction() {
            long ready = windows.stream().mapToLong(window -> window.readySamples).sum();
            long samplesCount = windows.stream().mapToLong(window -> window.sampleCount).sum();
            return samplesCount <= 0 ? Double.NaN : ((double) ready) / samplesCount;
        }

        double maxGapSec() {
            return windows.stream().mapToDouble(window -> window.maxGapSec).max().orElse(Double.NaN);
        }

        List<Double> collectAbsAimErrors() {
            return windows.stream().flatMap(window -> window.absAimErrorsDeg.stream()).collect(Collectors.toList());
        }

        List<Double> collectTeleopRequestedVsShotYawDiff() {
            return windows.stream()
                    .flatMap(window -> window.teleopRequestedVsShotYawDiffRadPerSec.stream())
                    .collect(Collectors.toList());
        }

        List<Double> collectTeleopHeadingConsistency() {
            return windows.stream().flatMap(window -> window.teleopHeadingConsistencyDeg.stream()).collect(Collectors.toList());
        }

        List<Double> collectAutoHeadingConsistency() {
            return windows.stream().flatMap(window -> window.autoHeadingConsistencyDeg.stream()).collect(Collectors.toList());
        }

        List<Double> collectShooterHeadingConsistency() {
            return windows.stream().flatMap(window -> window.shooterHeadingConsistencyDeg.stream()).collect(Collectors.toList());
        }

        long commandDirectionAgree() {
            return windows.stream().mapToLong(window -> window.commandDirectionAgree).sum();
        }

        long commandDirectionTotal() {
            return windows.stream().mapToLong(window -> window.commandDirectionTotal).sum();
        }

        long measuredDirectionAgree() {
            return windows.stream().mapToLong(window -> window.measuredDirectionAgree).sum();
        }

        long measuredDirectionTotal() {
            return windows.stream().mapToLong(window -> window.measuredDirectionTotal).sum();
        }

        List<String> suspiciousFindings() {
            List<String> findings = new ArrayList<>();
            long cancelAllDuringHeading = lifecycleEvents.stream()
                    .filter(event -> event.kind == LifecycleKind.CANCEL_ALL)
                    .filter(event -> windows.stream().anyMatch(window -> event.timestampUs >= window.startTsUs && event.timestampUs <= window.endTsUs))
                    .count();
            if (cancelAllDuringHeading > 0) {
                findings.add("cancelAllDuringHeading=" + cancelAllDuringHeading);
            }
            long interrupted = allCommandSessions().stream().filter(session -> session.interrupted).count();
            if (interrupted > 0) {
                findings.add("interruptedCommandSessions=" + interrupted);
            }
            long shortSessions = allCommandSessions().stream().filter(session -> session.durationSec() < SHORT_RUN_WARN_SEC).count();
            if (shortSessions > 0) {
                findings.add("shortCommandSessions=" + shortSessions);
            }
            long duplicateInstances = allCommandSessions().stream().filter(session -> session.maxActiveInstances > 1).count();
            if (duplicateInstances > 0) {
                findings.add("duplicateActiveInstances=" + duplicateInstances);
            }
            long badWindows = windows.stream().filter(window -> !window.issues.isEmpty()).count();
            if (badWindows > 0) {
                findings.add("windowsWithIssues=" + badWindows + "/" + windows.size());
            }
            return findings;
        }

        List<CommandSession> allCommandSessions() {
            List<CommandSession> sessions = new ArrayList<>();
            sessions.addAll(aimOnlySessions);
            sessions.addAll(shootSessions);
            return sessions;
        }
    }

    private static final class Aggregate {
        private final EnumMap<WindowKind, Integer> windowCounts = new EnumMap<>(WindowKind.class);
        private int totalSamples;
        private int totalWindows;
        private int totalAimOnlySessions;
        private int totalShootSessions;
        private int totalAutoPathSessions;
        private double totalWindowDurationSec;
        private double maxGapSec = Double.NaN;
        private int windowsWithGapWarn;
        private int windowsWithGapBad;
        private int interruptedCommandSessions;
        private int shortCommandSessions;
        private int duplicateActiveInstancesSessions;
        private int cancelAllDuringHeadingCount;
        private int readyWindows;
        private final List<Double> absAimErrorsDeg = new ArrayList<>();
        private final List<Double> teleopRequestedVsShotYawDiffRadPerSec = new ArrayList<>();
        private final List<Double> teleopHeadingConsistencyDeg = new ArrayList<>();
        private final List<Double> autoHeadingConsistencyDeg = new ArrayList<>();
        private final List<Double> shooterHeadingConsistencyDeg = new ArrayList<>();
        private long readySamples;
        private long totalActiveSamples;
        private long commandDirectionAgree;
        private long commandDirectionTotal;
        private long measuredDirectionAgree;
        private long measuredDirectionTotal;
        private long aimReadyViolationSamples;
        private long zeroCommandSamples;
        private long desiredHeadingMissingSamples;
        private final List<String> suspiciousFindings = new ArrayList<>();

        static Aggregate from(List<LogSummary> summaries) {
            Aggregate aggregate = new Aggregate();
            for (WindowKind kind : WindowKind.values()) {
                aggregate.windowCounts.put(kind, 0);
            }
            for (LogSummary summary : summaries) {
                aggregate.totalSamples += summary.samples.size();
                aggregate.totalWindows += summary.windows.size();
                aggregate.totalAimOnlySessions += summary.aimOnlySessions.size();
                aggregate.totalShootSessions += summary.shootSessions.size();
                aggregate.totalAutoPathSessions += summary.pathOverrideSessions.size();
                aggregate.totalWindowDurationSec += summary.totalWindowDurationSec();
                aggregate.maxGapSec = Double.isNaN(aggregate.maxGapSec)
                        ? summary.maxGapSec()
                        : Math.max(aggregate.maxGapSec, summary.maxGapSec());
                for (var entry : summary.windowCounts().entrySet()) {
                    aggregate.windowCounts.put(entry.getKey(), aggregate.windowCounts.get(entry.getKey()) + entry.getValue());
                }
                for (WindowSummary window : summary.windows) {
                    if (window.maxGapSec > GAP_WARN_SEC) {
                        aggregate.windowsWithGapWarn++;
                    }
                    if (window.maxGapSec > GAP_BAD_SEC) {
                        aggregate.windowsWithGapBad++;
                    }
                    if (window.readyFraction > 0.0) {
                        aggregate.readyWindows++;
                    }
                    aggregate.readySamples += window.readySamples;
                    aggregate.totalActiveSamples += window.sampleCount;
                    aggregate.absAimErrorsDeg.addAll(window.absAimErrorsDeg);
                    aggregate.teleopRequestedVsShotYawDiffRadPerSec.addAll(window.teleopRequestedVsShotYawDiffRadPerSec);
                    aggregate.teleopHeadingConsistencyDeg.addAll(window.teleopHeadingConsistencyDeg);
                    aggregate.autoHeadingConsistencyDeg.addAll(window.autoHeadingConsistencyDeg);
                    aggregate.shooterHeadingConsistencyDeg.addAll(window.shooterHeadingConsistencyDeg);
                    aggregate.commandDirectionAgree += window.commandDirectionAgree;
                    aggregate.commandDirectionTotal += window.commandDirectionTotal;
                    aggregate.measuredDirectionAgree += window.measuredDirectionAgree;
                    aggregate.measuredDirectionTotal += window.measuredDirectionTotal;
                    aggregate.aimReadyViolationSamples += window.aimReadyViolationSamples;
                    aggregate.zeroCommandSamples += window.zeroCommandAnomalySamples;
                    aggregate.desiredHeadingMissingSamples += window.desiredHeadingMissingSamples;
                }
                aggregate.interruptedCommandSessions += summary.allCommandSessions().stream().filter(session -> session.interrupted).count();
                aggregate.shortCommandSessions += summary.allCommandSessions().stream().filter(session -> session.durationSec() < SHORT_RUN_WARN_SEC).count();
                aggregate.duplicateActiveInstancesSessions += summary.allCommandSessions().stream().filter(session -> session.maxActiveInstances > 1).count();
                int cancelAllDuringHeading = (int) summary.lifecycleEvents.stream()
                        .filter(event -> event.kind == LifecycleKind.CANCEL_ALL)
                        .filter(event -> summary.windows.stream().anyMatch(window -> event.timestampUs >= window.startTsUs && event.timestampUs <= window.endTsUs))
                        .count();
                aggregate.cancelAllDuringHeadingCount += cancelAllDuringHeading;
                if (!summary.suspiciousFindings().isEmpty()) {
                    aggregate.suspiciousFindings.add("q" + summary.matchNumber + ": " + String.join(", ", summary.suspiciousFindings()));
                }
            }
            return aggregate;
        }

        double readyFraction() {
            return totalActiveSamples <= 0 ? Double.NaN : ((double) readySamples) / totalActiveSamples;
        }

        List<String> suspiciousFindings() {
            return suspiciousFindings;
        }
    }

    private record WindowRef(int matchNumber, Path wpilog, WindowSummary window) {}
}
