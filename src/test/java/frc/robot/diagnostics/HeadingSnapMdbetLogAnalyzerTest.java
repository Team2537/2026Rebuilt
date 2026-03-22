package frc.robot.diagnostics;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.util.struct.StructBuffer;
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
import java.util.Optional;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.junit.jupiter.api.Test;

class HeadingSnapMdbetLogAnalyzerTest {
    private static final Path LOG_DIR = Path.of("logs", "mdbet");
    private static final Path REPORT_PATH = Path.of("build", "reports", "diagnostics", "heading-snap-mdbet-summary.txt");
    private static final Path CSV_PATH = Path.of("build", "reports", "diagnostics", "heading-snap-mdbet-sessions.csv");
    private static final Pattern QUAL_MATCH_PATTERN = Pattern.compile(".*_q(\\d+)\\.wpilog");
    private static final Pattern COMMAND_EVENT_PATTERN = Pattern.compile(
            "(?<stamp>[0-9]+(?:\\.[0-9]+)?)s (?<kind>START|FINISH|INTERRUPT) run=(?<run>-?\\d+) name=(?<name>[^ ]+) source=(?<source>[^ ]+).*");
    private static final double RIGHT_X_ACTIVE_THRESHOLD = 0.1;
    private static final double SIGNIFICANT_ERROR_DEG = 5.0;
    private static final double SMALL_ERROR_DEG = 2.0;
    private static final double LOW_OMEGA_CMD_RAD_PER_SEC = 0.2;
    private static final double EVENT_NEAR_SEC = 0.20;
    private static final double POST_FINISH_WINDOW_SEC = 0.25;

    @Test
    void analyzeMdbetHeadingSnapAfterMatchSeven() throws Exception {
        List<Path> logs = findLogs();
        assertFalse(logs.isEmpty(), "No mdbet qualification logs found after q7 in " + LOG_DIR.toAbsolutePath());

        List<LogReport> reports = new ArrayList<>();
        for (Path log : logs) {
            reports.add(analyzeLog(log));
        }

        String reportText = formatReport(reports);
        String csvText = formatCsv(reports);
        Files.createDirectories(REPORT_PATH.getParent());
        Files.writeString(REPORT_PATH, reportText);
        Files.writeString(CSV_PATH, csvText);

        System.out.println(reportText);
        System.out.println("Heading snap CSV written: " + CSV_PATH.toAbsolutePath());

        for (LogReport report : reports) {
            assertTrue(report.entriesFound.coreEntriesPresent(), "Missing core heading-snap entries in " + report.logPath);
        }
    }

    private static List<Path> findLogs() throws IOException {
        try (var stream = Files.list(LOG_DIR)) {
            return stream.filter(Files::isRegularFile)
                    .filter(path -> {
                        Matcher matcher = QUAL_MATCH_PATTERN.matcher(path.getFileName().toString());
                        return matcher.matches() && Integer.parseInt(matcher.group(1)) > 7;
                    })
                    .sorted(Comparator.comparing(Path::toString))
                    .toList();
        }
    }

    private static LogReport analyzeLog(Path logPath) throws IOException {
        Map<Integer, EntryInfo> entries = scanEntries(logPath);
        EntryIds ids = EntryIds.from(entries);

        var poseBuffer = StructBuffer.create(Pose2d.struct);
        var speedsBuffer = StructBuffer.create(ChassisSpeeds.struct);
        CurrentState state = new CurrentState();
        List<StateSample> samples = new ArrayList<>();
        List<CommandEvent> events = new ArrayList<>();
        List<RunningTransition> snapRunningTransitions = new ArrayList<>();
        List<RunningTransition> driveDefaultRunningTransitions = new ArrayList<>();
        List<String> previousRecentEvents = List.of();
        Map<String, Boolean> seenRawEvents = new LinkedHashMap<>();

        DataLogReader reader = new DataLogReader(logPath.toString());
        if (!reader.isValid()) {
            throw new IllegalStateException("Invalid WPILOG file: " + logPath.toAbsolutePath());
        }

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

            double timeSec = record.getTimestamp() / 1_000_000.0;
            int entry = record.getEntry();
            boolean relevant = false;
            boolean controlUpdate = false;
            SampleKind sampleKind = SampleKind.OTHER;

            if (entry == ids.poseEntry) {
                state.pose = poseBuffer.read(record.getRaw());
                relevant = true;
                sampleKind = SampleKind.POSE;
            } else if (entry == ids.requestedSpeedsEntry) {
                state.requestedSpeeds = speedsBuffer.read(record.getRaw());
                relevant = true;
                sampleKind = SampleKind.REQUESTED_SPEEDS;
            } else if (entry == ids.measuredSpeedsEntry) {
                state.measuredSpeeds = speedsBuffer.read(record.getRaw());
                relevant = true;
                sampleKind = SampleKind.MEASURED_SPEEDS;
            } else if (entry == ids.enabledEntry) {
                state.enabled = record.getBoolean();
                relevant = true;
                sampleKind = SampleKind.ENABLED;
            } else if (entry == ids.modeEntry) {
                state.mode = record.getString();
                relevant = true;
                sampleKind = SampleKind.MODE;
            } else if (entry == ids.matchTimeEntry) {
                state.matchTime = record.getDouble();
                relevant = true;
                sampleKind = SampleKind.MATCH_TIME;
            } else if (entry == ids.allianceEntry) {
                state.alliance = record.getString();
                relevant = true;
                sampleKind = SampleKind.ALLIANCE;
            } else if (entry == ids.gyroConnectedEntry) {
                state.gyroConnected = record.getBoolean();
                relevant = true;
                sampleKind = SampleKind.GYRO_CONNECTED;
            } else if (entry == ids.axesEntry) {
                double[] axes = readDoubleArrayLenient(record);
                state.rightX = axes != null && axes.length > 4 ? axes[4] : Double.NaN;
                state.leftX = axes != null && axes.length > 0 ? axes[0] : Double.NaN;
                state.leftY = axes != null && axes.length > 1 ? axes[1] : Double.NaN;
                relevant = true;
                sampleKind = SampleKind.AXES;
            } else if (entry == ids.desiredHeadingEntry) {
                state.desiredHeadingDeg = record.getDouble();
                relevant = true;
                controlUpdate = true;
                sampleKind = SampleKind.DESIRED_HEADING;
            } else if (entry == ids.headingErrorEntry) {
                state.headingErrorDeg = record.getDouble();
                relevant = true;
                controlUpdate = true;
                sampleKind = SampleKind.HEADING_ERROR;
            } else if (entry == ids.desiredRateEntry) {
                state.desiredRateRadPerSec = record.getDouble();
                relevant = true;
                controlUpdate = true;
                sampleKind = SampleKind.DESIRED_RATE;
            } else if (entry == ids.measuredOmegaEntry) {
                state.measuredOmegaRadPerSec = record.getDouble();
                relevant = true;
                controlUpdate = true;
                sampleKind = SampleKind.MEASURED_OMEGA;
            } else if (entry == ids.feedbackOmegaEntry) {
                state.feedbackOmegaRadPerSec = record.getDouble();
                relevant = true;
                controlUpdate = true;
                sampleKind = SampleKind.FEEDBACK_OMEGA;
            } else if (entry == ids.omegaCommandEntry) {
                state.omegaCommandRadPerSec = record.getDouble();
                relevant = true;
                controlUpdate = true;
                sampleKind = SampleKind.OMEGA_COMMAND;
            } else if (entry == ids.snapRunningEntry) {
                boolean newValue = readBooleanLenient(record);
                if (state.snapRunning == null || newValue != state.snapRunning.booleanValue()) {
                    snapRunningTransitions.add(new RunningTransition(timeSec, newValue));
                }
                state.snapRunning = newValue;
                relevant = true;
                sampleKind = SampleKind.SNAP_RUNNING;
            } else if (entry == ids.driveDefaultRunningEntry) {
                boolean newValue = readBooleanLenient(record);
                if (state.driveDefaultRunning == null || newValue != state.driveDefaultRunning.booleanValue()) {
                    driveDefaultRunningTransitions.add(new RunningTransition(timeSec, newValue));
                }
                state.driveDefaultRunning = newValue;
                relevant = true;
                sampleKind = SampleKind.DRIVE_DEFAULT_RUNNING;
            } else if (entry == ids.recentEventsEntry) {
                List<String> currentRecentEvents = readStringArrayLenient(record);
                for (String rawEvent : appendedEvents(previousRecentEvents, currentRecentEvents)) {
                    if (!seenRawEvents.containsKey(rawEvent)) {
                        parseCommandEvent(rawEvent).ifPresent(events::add);
                        seenRawEvents.put(rawEvent, Boolean.TRUE);
                    }
                }
                previousRecentEvents = currentRecentEvents;
                relevant = true;
                sampleKind = SampleKind.RECENT_EVENTS;
            } else if (entry == ids.lastEventEntry) {
                String rawEvent = record.getString();
                if (!seenRawEvents.containsKey(rawEvent)) {
                    parseCommandEvent(rawEvent).ifPresent(events::add);
                    seenRawEvents.put(rawEvent, Boolean.TRUE);
                }
                relevant = true;
                sampleKind = SampleKind.LAST_EVENT;
            }

            if (relevant) {
                samples.add(state.snapshot(timeSec, controlUpdate, sampleKind));
            }
        }

        List<SnapSession> sessions = buildSessions(logPath, samples, events, snapRunningTransitions, driveDefaultRunningTransitions);
        return new LogReport(logPath.toAbsolutePath(), matchLabel(logPath), ids.presentSummary(), events, snapRunningTransitions,
                driveDefaultRunningTransitions, sessions);
    }

    private static Map<Integer, EntryInfo> scanEntries(Path logPath) throws IOException {
        DataLogReader reader = new DataLogReader(logPath.toString());
        if (!reader.isValid()) {
            throw new IllegalStateException("Invalid WPILOG file: " + logPath.toAbsolutePath());
        }
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

    private static List<SnapSession> buildSessions(
            Path logPath,
            List<StateSample> samples,
            List<CommandEvent> events,
            List<RunningTransition> snapRunningTransitions,
            List<RunningTransition> driveDefaultRunningTransitions) {
        if (!snapRunningTransitions.isEmpty()) {
            return buildSessionsFromRunningTransitions(logPath, samples, events, snapRunningTransitions, driveDefaultRunningTransitions);
        }

        Map<Integer, CommandEvent> startsByRun = new LinkedHashMap<>();
        List<SnapSession> sessions = new ArrayList<>();
        for (CommandEvent event : events) {
            if (!"DriveHeadingSnap".equals(event.name)) {
                continue;
            }
            if (event.kind == EventKind.START) {
                startsByRun.put(event.runId, event);
                continue;
            }
            if (event.kind == EventKind.FINISH || event.kind == EventKind.INTERRUPT) {
                CommandEvent start = startsByRun.remove(event.runId);
                if (start != null) {
                    sessions.add(analyzeSession(logPath, start, event, samples, events, snapRunningTransitions, driveDefaultRunningTransitions));
                }
            }
        }
        for (CommandEvent danglingStart : startsByRun.values()) {
            sessions.add(analyzeDanglingSession(logPath, danglingStart, samples, events, snapRunningTransitions, driveDefaultRunningTransitions));
        }
        return sessions;
    }

    private static List<SnapSession> buildSessionsFromRunningTransitions(
            Path logPath,
            List<StateSample> samples,
            List<CommandEvent> events,
            List<RunningTransition> snapRunningTransitions,
            List<RunningTransition> driveDefaultRunningTransitions) {
        List<SnapSession> sessions = new ArrayList<>();
        RunningTransition currentStart = null;
        for (RunningTransition transition : snapRunningTransitions) {
            if (transition.running) {
                currentStart = transition;
                continue;
            }
            if (currentStart == null) {
                continue;
            }

            CommandEvent startEvent = nearestCommandEvent(events, "DriveHeadingSnap", EventKind.START, currentStart.timeSec, EVENT_NEAR_SEC)
                    .orElse(new CommandEvent(currentStart.timeSec, EventKind.START, -1, "DriveHeadingSnap", "unknown", "<synthetic start>"));
            CommandEvent endEvent = nearestEndEventForRun(events, startEvent.runId, transition.timeSec, EVENT_NEAR_SEC)
                    .orElseGet(() -> nearestEitherEndEvent(events, transition.timeSec, EVENT_NEAR_SEC)
                            .orElse(new CommandEvent(transition.timeSec, EventKind.INTERRUPT, startEvent.runId, "DriveHeadingSnap", startEvent.source, "<synthetic end>")));
            sessions.add(analyzeSession(logPath, startEvent.withTime(currentStart.timeSec), endEvent.withTime(transition.timeSec),
                    samples, events, snapRunningTransitions, driveDefaultRunningTransitions));
            currentStart = null;
        }

        if (currentStart != null) {
            CommandEvent startEvent = nearestCommandEvent(events, "DriveHeadingSnap", EventKind.START, currentStart.timeSec, EVENT_NEAR_SEC)
                    .orElse(new CommandEvent(currentStart.timeSec, EventKind.START, -1, "DriveHeadingSnap", "unknown", "<synthetic start>"));
            sessions.add(analyzeDanglingSession(logPath, startEvent.withTime(currentStart.timeSec), samples, events,
                    snapRunningTransitions, driveDefaultRunningTransitions));
        }

        return sessions;
    }

    private static SnapSession analyzeSession(
            Path logPath,
            CommandEvent start,
            CommandEvent end,
            List<StateSample> samples,
            List<CommandEvent> events,
            List<RunningTransition> snapRunningTransitions,
            List<RunningTransition> driveDefaultRunningTransitions) {
        double startSec = start.timeSec;
        double endSec = end.timeSec;
        double postEndLimitSec = nextDriveHeadingSnapStart(events, endSec).map(next -> Math.min(endSec + POST_FINISH_WINDOW_SEC, next - 1e-3))
                .orElse(endSec + POST_FINISH_WINDOW_SEC);
        List<StateSample> during = samplesBetween(samples, startSec, endSec);
        List<StateSample> post = samplesBetween(samples, endSec, postEndLimitSec);
        StateSample before = sampleAtOrBefore(samples, startSec);
        StateSample atEnd = sampleAtOrBefore(samples, endSec);

        double startHeadingDeg = before != null ? before.robotHeadingDeg() : Double.NaN;
        double endHeadingDeg = atEnd != null ? atEnd.robotHeadingDeg() : Double.NaN;
        String alliance = before != null ? before.alliance : "";
        double targetHeadingDeg = lastFiniteDesiredHeading(during);
        List<StateSample> analyzed = samplesAfterTargetLatch(during, targetHeadingDeg);
        double targetHeadingSpanDeg = maxDesiredHeadingSpan(analyzed, targetHeadingDeg);
        double targetCardinalErrorDeg = nearestCardinalErrorDeg(targetHeadingDeg);
        double expectedTargetHeadingDeg = expectedHeadingForSource(start.source, alliance);
        double expectedTargetErrorDeg = Double.isNaN(expectedTargetHeadingDeg)
                ? Double.NaN
                : angleDiffDeg(targetHeadingDeg, expectedTargetHeadingDeg);

        double maxAbsErrorDeg = 0.0;
        double meanAbsErrorDeg = 0.0;
        int errorSamples = 0;
        double finalAbsErrorDeg = Double.NaN;
        double bestAbsErrorDeg = Double.POSITIVE_INFINITY;
        double settleTimeTo5DegSec = Double.NaN;
        double settleTimeTo2DegSec = Double.NaN;
        double maxAbsDesiredRateRadPerSec = 0.0;
        double maxAbsFeedbackMinusCommand = 0.0;
        double maxAbsOmegaCommandRadPerSec = 0.0;
        double maxAbsMeasuredOmegaRadPerSec = 0.0;
        double maxLinearSpeedMps = 0.0;
        double maxRequestedLinearSpeedMps = 0.0;
        double maxRightXAbs = 0.0;
        double endWindowRightXAbs = 0.0;
        double maxControlGapSec = 0.0;
        int invalidDesiredHeadingSamples = 0;
        int invalidHeadingErrorSamples = 0;
        int invalidOmegaCommandSamples = 0;
        int largeErrorLowCommandSamples = 0;
        int wrongWayCommandSamples = 0;
        int measuredOmegaOpposesCommandSamples = 0;
        int significantErrorSignFlips = 0;

        Double previousControlTime = null;
        Double previousErrorSign = null;

        for (StateSample sample : analyzed) {
            maxRightXAbs = Math.max(maxRightXAbs, absOrZero(sample.rightX));
            if (sample.timeSec >= endSec - 0.10) {
                endWindowRightXAbs = Math.max(endWindowRightXAbs, absOrZero(sample.rightX));
            }
            maxLinearSpeedMps = Math.max(maxLinearSpeedMps, sample.measuredLinearSpeed());
            maxRequestedLinearSpeedMps = Math.max(maxRequestedLinearSpeedMps, sample.requestedLinearSpeed());

            if (sample.controlUpdate) {
                if (previousControlTime != null) {
                    maxControlGapSec = Math.max(maxControlGapSec, sample.timeSec - previousControlTime);
                }
                previousControlTime = sample.timeSec;
            }

            if (!Double.isFinite(sample.desiredHeadingDeg)) {
                invalidDesiredHeadingSamples++;
            }
            if (!Double.isFinite(sample.headingErrorDeg)) {
                invalidHeadingErrorSamples++;
            }
            if (!Double.isFinite(sample.omegaCommandRadPerSec)) {
                invalidOmegaCommandSamples++;
            }
            if (Double.isFinite(sample.headingErrorDeg)) {
                double absError = Math.abs(sample.headingErrorDeg);
                maxAbsErrorDeg = Math.max(maxAbsErrorDeg, absError);
                meanAbsErrorDeg += absError;
                errorSamples++;
                bestAbsErrorDeg = Math.min(bestAbsErrorDeg, absError);
                finalAbsErrorDeg = absError;
                if (Double.isNaN(settleTimeTo5DegSec) && absError <= SIGNIFICANT_ERROR_DEG) {
                    settleTimeTo5DegSec = sample.timeSec - startSec;
                }
                if (Double.isNaN(settleTimeTo2DegSec) && absError <= SMALL_ERROR_DEG) {
                    settleTimeTo2DegSec = sample.timeSec - startSec;
                }
                double currentSign = Math.signum(sample.headingErrorDeg);
                if (absError >= SMALL_ERROR_DEG && currentSign != 0.0) {
                    if (previousErrorSign != null && currentSign != previousErrorSign.doubleValue()) {
                        significantErrorSignFlips++;
                    }
                    previousErrorSign = currentSign;
                }
            }
            if (Double.isFinite(sample.desiredRateRadPerSec)) {
                maxAbsDesiredRateRadPerSec = Math.max(maxAbsDesiredRateRadPerSec, Math.abs(sample.desiredRateRadPerSec));
            }
            if (sample.sampleKind == SampleKind.OMEGA_COMMAND
                    && Double.isFinite(sample.feedbackOmegaRadPerSec)
                    && Double.isFinite(sample.omegaCommandRadPerSec)) {
                maxAbsFeedbackMinusCommand = Math.max(
                        maxAbsFeedbackMinusCommand,
                        Math.abs(sample.feedbackOmegaRadPerSec - sample.omegaCommandRadPerSec));
            }
            if (Double.isFinite(sample.omegaCommandRadPerSec)) {
                maxAbsOmegaCommandRadPerSec = Math.max(maxAbsOmegaCommandRadPerSec, Math.abs(sample.omegaCommandRadPerSec));
            }
            if (Double.isFinite(sample.measuredOmegaRadPerSec)) {
                maxAbsMeasuredOmegaRadPerSec = Math.max(maxAbsMeasuredOmegaRadPerSec, Math.abs(sample.measuredOmegaRadPerSec));
            }

            if (Double.isFinite(sample.headingErrorDeg) && Double.isFinite(sample.omegaCommandRadPerSec)
                    && sample.sampleKind == SampleKind.OMEGA_COMMAND
                    && Math.abs(sample.headingErrorDeg) > SIGNIFICANT_ERROR_DEG
                    && Math.abs(sample.omegaCommandRadPerSec) < LOW_OMEGA_CMD_RAD_PER_SEC) {
                largeErrorLowCommandSamples++;
            }
            if (Double.isFinite(sample.headingErrorDeg) && Double.isFinite(sample.omegaCommandRadPerSec)
                    && sample.sampleKind == SampleKind.OMEGA_COMMAND
                    && Math.abs(sample.headingErrorDeg) > SIGNIFICANT_ERROR_DEG
                    && Math.abs(sample.omegaCommandRadPerSec) >= LOW_OMEGA_CMD_RAD_PER_SEC
                    && Math.signum(sample.headingErrorDeg) != Math.signum(sample.omegaCommandRadPerSec)) {
                wrongWayCommandSamples++;
            }
            if (Double.isFinite(sample.measuredOmegaRadPerSec) && Double.isFinite(sample.omegaCommandRadPerSec)
                    && sample.sampleKind == SampleKind.OMEGA_COMMAND
                    && Math.abs(sample.omegaCommandRadPerSec) > 0.75
                    && Math.signum(sample.measuredOmegaRadPerSec) != 0.0
                    && Math.signum(sample.measuredOmegaRadPerSec) != Math.signum(sample.omegaCommandRadPerSec)) {
                measuredOmegaOpposesCommandSamples++;
            }
        }
        if (errorSamples > 0) {
            meanAbsErrorDeg /= errorSamples;
        }
        if (bestAbsErrorDeg == Double.POSITIVE_INFINITY) {
            bestAbsErrorDeg = Double.NaN;
        }

        double postFinishMaxAbsErrorDeg = maxAbsError(post);
        double postFinishMaxAbsMeasuredOmegaRadPerSec = maxAbsMeasuredOmega(post);
        boolean sawSnapRunningTrue = sawTransitionNear(snapRunningTransitions, true, startSec, EVENT_NEAR_SEC);
        boolean sawSnapRunningFalse = sawTransitionNear(snapRunningTransitions, false, endSec, EVENT_NEAR_SEC);
        boolean driveDefaultInterruptedNearStart = sawTransitionNear(driveDefaultRunningTransitions, false, startSec, EVENT_NEAR_SEC);
        boolean driveDefaultRestartedNearEnd = sawTransitionNear(driveDefaultRunningTransitions, true, endSec, EVENT_NEAR_SEC);
        List<CommandEvent> nearbyEndEvents = commandEventsNear(events, endSec, EVENT_NEAR_SEC);

        String termination = classifyTermination(
                end.kind,
                endWindowRightXAbs,
                atEnd,
                nearbyEndEvents,
                finalAbsErrorDeg,
                postFinishMaxAbsErrorDeg,
                postFinishMaxAbsMeasuredOmegaRadPerSec);
        List<String> anomalies = new ArrayList<>();
        if (end.kind == EventKind.INTERRUPT && !"driverTurnInput".equals(termination) && !"robotDisabled".equals(termination)) {
            anomalies.add("unexpectedInterrupt");
        }
        if (end.kind == EventKind.FINISH && !"settled".equals(termination)) {
            anomalies.add(termination);
        }
        if (!Double.isFinite(targetHeadingDeg)) {
            anomalies.add("missingTargetHeading");
        }
        if (targetHeadingSpanDeg > 2.0) {
            anomalies.add(String.format(Locale.US, "targetMoved(%.2fdeg)", targetHeadingSpanDeg));
        }
        if (targetCardinalErrorDeg > 0.5) {
            anomalies.add(String.format(Locale.US, "targetNotCardinal(%.2fdeg)", targetCardinalErrorDeg));
        }
        if (Double.isFinite(expectedTargetErrorDeg) && expectedTargetErrorDeg > 0.5) {
            anomalies.add(String.format(Locale.US, "sourceTargetMismatch(%.2fdeg)", expectedTargetErrorDeg));
        }
        if (maxAbsDesiredRateRadPerSec > 0.05) {
            anomalies.add(String.format(Locale.US, "nonzeroDesiredRate(%.4f)", maxAbsDesiredRateRadPerSec));
        }
        if (maxAbsFeedbackMinusCommand > 1e-6) {
            anomalies.add(String.format(Locale.US, "feedbackCmdMismatch(%.4f)", maxAbsFeedbackMinusCommand));
        }
        if (invalidDesiredHeadingSamples > 0 || invalidHeadingErrorSamples > 0 || invalidOmegaCommandSamples > 0) {
            anomalies.add(String.format(Locale.US,
                    "invalidSamples(target=%d,error=%d,cmd=%d)",
                    invalidDesiredHeadingSamples,
                    invalidHeadingErrorSamples,
                    invalidOmegaCommandSamples));
        }
        if (largeErrorLowCommandSamples > 5) {
            anomalies.add("largeErrorLowCmd=" + largeErrorLowCommandSamples);
        }
        if (wrongWayCommandSamples > 3) {
            anomalies.add("wrongWayCmd=" + wrongWayCommandSamples);
        }
        if (measuredOmegaOpposesCommandSamples > 20) {
            anomalies.add("measuredOpposesCmd=" + measuredOmegaOpposesCommandSamples);
        }
        if (maxControlGapSec > 0.06) {
            anomalies.add(String.format(Locale.US, "controlGap=%.3fs", maxControlGapSec));
        }
        if (!sawSnapRunningTrue || !sawSnapRunningFalse) {
            anomalies.add("snapRunningTelemetryMismatch");
        }
        if (!driveDefaultInterruptedNearStart) {
            anomalies.add("driveDefaultNotInterruptedNearStart");
        }
        if (!driveDefaultRestartedNearEnd) {
            anomalies.add("driveDefaultNotRestartedNearEnd");
        }
        if (Double.isFinite(finalAbsErrorDeg) && finalAbsErrorDeg > 2.0 && end.kind == EventKind.FINISH) {
            anomalies.add(String.format(Locale.US, "finishError=%.2fdeg", finalAbsErrorDeg));
        }
        if (Double.isFinite(postFinishMaxAbsErrorDeg) && postFinishMaxAbsErrorDeg > 2.5 && end.kind == EventKind.FINISH) {
            anomalies.add(String.format(Locale.US, "postFinishError=%.2fdeg", postFinishMaxAbsErrorDeg));
        }
        if (Double.isFinite(postFinishMaxAbsMeasuredOmegaRadPerSec)
                && postFinishMaxAbsMeasuredOmegaRadPerSec > 0.6
                && end.kind == EventKind.FINISH) {
            anomalies.add(String.format(Locale.US, "postFinishOmega=%.2f", postFinishMaxAbsMeasuredOmegaRadPerSec));
        }

        return new SnapSession(
                logPath.getFileName().toString(),
                start.runId,
                start.source,
                startSec,
                endSec,
                endSec - startSec,
                end.kind,
                termination,
                startHeadingDeg,
                endHeadingDeg,
                targetHeadingDeg,
                targetHeadingSpanDeg,
                targetCardinalErrorDeg,
                expectedTargetHeadingDeg,
                expectedTargetErrorDeg,
                maxAbsErrorDeg,
                meanAbsErrorDeg,
                bestAbsErrorDeg,
                finalAbsErrorDeg,
                settleTimeTo5DegSec,
                settleTimeTo2DegSec,
                significantErrorSignFlips,
                maxAbsDesiredRateRadPerSec,
                maxAbsFeedbackMinusCommand,
                maxAbsOmegaCommandRadPerSec,
                maxAbsMeasuredOmegaRadPerSec,
                maxLinearSpeedMps,
                maxRequestedLinearSpeedMps,
                maxRightXAbs,
                endWindowRightXAbs,
                maxControlGapSec,
                invalidDesiredHeadingSamples,
                invalidHeadingErrorSamples,
                invalidOmegaCommandSamples,
                largeErrorLowCommandSamples,
                wrongWayCommandSamples,
                measuredOmegaOpposesCommandSamples,
                sawSnapRunningTrue,
                sawSnapRunningFalse,
                driveDefaultInterruptedNearStart,
                driveDefaultRestartedNearEnd,
                postFinishMaxAbsErrorDeg,
                postFinishMaxAbsMeasuredOmegaRadPerSec,
                nearbySummary(nearbyEndEvents, start.runId),
                anomalies);
    }

    private static SnapSession analyzeDanglingSession(
            Path logPath,
            CommandEvent start,
            List<StateSample> samples,
            List<CommandEvent> events,
            List<RunningTransition> snapRunningTransitions,
            List<RunningTransition> driveDefaultRunningTransitions) {
        CommandEvent syntheticEnd = new CommandEvent(start.timeSec, EventKind.INTERRUPT, start.runId, start.name, start.source, "<missing end>");
        SnapSession session = analyzeSession(logPath, start, syntheticEnd, samples, events, snapRunningTransitions, driveDefaultRunningTransitions);
        List<String> anomalies = new ArrayList<>(session.anomalies);
        anomalies.add("missingEndEvent");
        return session.withAnomalies(anomalies);
    }

    private static Optional<CommandEvent> parseCommandEvent(String rawEvent) {
        Matcher matcher = COMMAND_EVENT_PATTERN.matcher(rawEvent);
        if (!matcher.matches()) {
            return Optional.empty();
        }
        return Optional.of(new CommandEvent(
                Double.parseDouble(matcher.group("stamp")),
                EventKind.valueOf(matcher.group("kind")),
                Integer.parseInt(matcher.group("run")),
                matcher.group("name"),
                matcher.group("source"),
                rawEvent));
    }

    private static List<String> appendedEvents(List<String> previous, List<String> current) {
        if (current == null || current.isEmpty()) {
            return List.of();
        }
        if (previous == null || previous.isEmpty()) {
            return current;
        }

        int maxOverlap = Math.min(previous.size(), current.size());
        for (int overlap = maxOverlap; overlap >= 0; overlap--) {
            boolean matches = true;
            for (int i = 0; i < overlap; i++) {
                if (!previous.get(previous.size() - overlap + i).equals(current.get(i))) {
                    matches = false;
                    break;
                }
            }
            if (matches) {
                return current.subList(overlap, current.size());
            }
        }
        return current;
    }

    private static List<StateSample> samplesBetween(List<StateSample> samples, double startSec, double endSec) {
        List<StateSample> result = new ArrayList<>();
        for (StateSample sample : samples) {
            if (sample.timeSec < startSec) {
                continue;
            }
            if (sample.timeSec > endSec) {
                break;
            }
            result.add(sample);
        }
        return result;
    }

    private static StateSample sampleAtOrBefore(List<StateSample> samples, double timeSec) {
        StateSample candidate = null;
        for (StateSample sample : samples) {
            if (sample.timeSec > timeSec) {
                break;
            }
            candidate = sample;
        }
        return candidate;
    }

    private static boolean sawTransitionNear(List<RunningTransition> transitions, boolean state, double centerSec, double windowSec) {
        for (RunningTransition transition : transitions) {
            if (transition.running == state && Math.abs(transition.timeSec - centerSec) <= windowSec) {
                return true;
            }
        }
        return false;
    }

    private static List<CommandEvent> commandEventsNear(List<CommandEvent> events, double centerSec, double windowSec) {
        List<CommandEvent> result = new ArrayList<>();
        for (CommandEvent event : events) {
            if (Math.abs(event.timeSec - centerSec) <= windowSec) {
                result.add(event);
            }
        }
        return result;
    }

    private static Optional<Double> nextDriveHeadingSnapStart(List<CommandEvent> events, double afterSec) {
        Double best = null;
        for (CommandEvent event : events) {
            if (!"DriveHeadingSnap".equals(event.name) || event.kind != EventKind.START || event.timeSec <= afterSec) {
                continue;
            }
            if (best == null || event.timeSec < best.doubleValue()) {
                best = event.timeSec;
            }
        }
        return Optional.ofNullable(best);
    }

    private static Optional<CommandEvent> nearestCommandEvent(
            List<CommandEvent> events,
            String name,
            EventKind kind,
            double centerSec,
            double windowSec) {
        CommandEvent best = null;
        double bestDt = Double.POSITIVE_INFINITY;
        for (CommandEvent event : events) {
            if (!name.equals(event.name) || event.kind != kind) {
                continue;
            }
            double dt = Math.abs(event.timeSec - centerSec);
            if (dt <= windowSec && dt < bestDt) {
                best = event;
                bestDt = dt;
            }
        }
        return Optional.ofNullable(best);
    }

    private static Optional<CommandEvent> nearestEitherEndEvent(List<CommandEvent> events, double centerSec, double windowSec) {
        CommandEvent best = null;
        double bestDt = Double.POSITIVE_INFINITY;
        for (CommandEvent event : events) {
            if (!"DriveHeadingSnap".equals(event.name)) {
                continue;
            }
            if (event.kind != EventKind.FINISH && event.kind != EventKind.INTERRUPT) {
                continue;
            }
            double dt = Math.abs(event.timeSec - centerSec);
            if (dt <= windowSec && dt < bestDt) {
                best = event;
                bestDt = dt;
            }
        }
        return Optional.ofNullable(best);
    }

    private static Optional<CommandEvent> nearestEndEventForRun(
            List<CommandEvent> events,
            int runId,
            double centerSec,
            double windowSec) {
        if (runId < 0) {
            return Optional.empty();
        }
        CommandEvent best = null;
        double bestDt = Double.POSITIVE_INFINITY;
        for (CommandEvent event : events) {
            if (!"DriveHeadingSnap".equals(event.name) || event.runId != runId) {
                continue;
            }
            if (event.kind != EventKind.FINISH && event.kind != EventKind.INTERRUPT) {
                continue;
            }
            double dt = Math.abs(event.timeSec - centerSec);
            if (dt <= windowSec && dt < bestDt) {
                best = event;
                bestDt = dt;
            }
        }
        return Optional.ofNullable(best);
    }

    private static String nearbySummary(List<CommandEvent> events, int runId) {
        List<String> pieces = new ArrayList<>();
        for (CommandEvent event : events) {
            if ("DriveHeadingSnap".equals(event.name) && event.runId == runId) {
                continue;
            }
            pieces.add(String.format(Locale.US, "%.3f:%s:%s:%s", event.timeSec, event.kind, event.name, event.source));
        }
        return pieces.isEmpty() ? "" : String.join(" | ", pieces);
    }

    private static String classifyTermination(
            EventKind endKind,
            double endWindowRightXAbs,
            StateSample atEnd,
            List<CommandEvent> nearbyEndEvents,
            double finalAbsErrorDeg,
            double postFinishAbsErrorDeg,
            double postFinishAbsMeasuredOmegaRadPerSec) {
        if (endKind == EventKind.FINISH) {
            if (endWindowRightXAbs > RIGHT_X_ACTIVE_THRESHOLD) {
                return "driverTurnInput";
            }
            boolean errorSettled = Double.isFinite(finalAbsErrorDeg) && finalAbsErrorDeg <= 2.0;
            boolean postErrorSettled = !Double.isFinite(postFinishAbsErrorDeg) || postFinishAbsErrorDeg <= 2.5;
            boolean postOmegaSettled = !Double.isFinite(postFinishAbsMeasuredOmegaRadPerSec) || postFinishAbsMeasuredOmegaRadPerSec <= 0.6;
            return errorSettled && postErrorSettled && postOmegaSettled ? "settled" : "unresolvedFinish";
        }
        if (endWindowRightXAbs > RIGHT_X_ACTIVE_THRESHOLD) {
            return "driverTurnInput";
        }
        if (atEnd != null && !atEnd.enabled) {
            return "robotDisabled";
        }
        for (CommandEvent event : nearbyEndEvents) {
            if (event.kind == EventKind.START && !"DriveHeadingSnap".equals(event.name) && !"DriveJoystickDefault".equals(event.name)) {
                return "preemptedBy" + event.name;
            }
        }
        return "unknownInterrupt";
    }

    private static double lastFiniteDesiredHeading(List<StateSample> samples) {
        for (int i = samples.size() - 1; i >= 0; i--) {
            StateSample sample = samples.get(i);
            if (Double.isFinite(sample.desiredHeadingDeg)) {
                return sample.desiredHeadingDeg;
            }
        }
        return Double.NaN;
    }

    private static List<StateSample> samplesAfterTargetLatch(List<StateSample> samples, double targetHeadingDeg) {
        if (!Double.isFinite(targetHeadingDeg)) {
            return samples;
        }
        int startIndex = -1;
        for (int i = 0; i < samples.size(); i++) {
            StateSample sample = samples.get(i);
            if (Double.isFinite(sample.desiredHeadingDeg) && angleDiffDeg(sample.desiredHeadingDeg, targetHeadingDeg) <= 5.0) {
                startIndex = i;
                break;
            }
        }
        if (startIndex < 0) {
            return samples;
        }
        return samples.subList(startIndex, samples.size());
    }

    private static double maxDesiredHeadingSpan(List<StateSample> samples, double referenceDeg) {
        if (!Double.isFinite(referenceDeg)) {
            return Double.NaN;
        }
        double max = 0.0;
        for (StateSample sample : samples) {
            if (Double.isFinite(sample.desiredHeadingDeg)) {
                max = Math.max(max, angleDiffDeg(sample.desiredHeadingDeg, referenceDeg));
            }
        }
        return max;
    }

    private static double nearestCardinalErrorDeg(double headingDeg) {
        if (!Double.isFinite(headingDeg)) {
            return Double.NaN;
        }
        double snapped = Math.round(headingDeg / 90.0) * 90.0;
        return angleDiffDeg(headingDeg, snapped);
    }

    private static double expectedHeadingForSource(String source, String alliance) {
        double blueHeadingDeg;
        switch (source) {
            case "driver.a.onTrue" -> blueHeadingDeg = 180.0;
            case "driver.b.onTrue" -> blueHeadingDeg = -90.0;
            case "driver.x.onTrue" -> blueHeadingDeg = 90.0;
            case "driver.y.onTrue" -> blueHeadingDeg = 0.0;
            default -> {
                return Double.NaN;
            }
        }
        if ("Red".equalsIgnoreCase(alliance)) {
            blueHeadingDeg += 180.0;
        }
        return normalizeDeg(blueHeadingDeg);
    }

    private static double maxAbsError(List<StateSample> samples) {
        double max = Double.NaN;
        for (StateSample sample : samples) {
            if (Double.isFinite(sample.headingErrorDeg)) {
                max = Double.isNaN(max) ? Math.abs(sample.headingErrorDeg) : Math.max(max, Math.abs(sample.headingErrorDeg));
            }
        }
        return max;
    }

    private static double maxAbsMeasuredOmega(List<StateSample> samples) {
        double max = Double.NaN;
        for (StateSample sample : samples) {
            if (Double.isFinite(sample.measuredOmegaRadPerSec)) {
                max = Double.isNaN(max)
                        ? Math.abs(sample.measuredOmegaRadPerSec)
                        : Math.max(max, Math.abs(sample.measuredOmegaRadPerSec));
            }
        }
        return max;
    }

    private static double angleDiffDeg(double aDeg, double bDeg) {
        return Math.abs(normalizeDeg(aDeg - bDeg));
    }

    private static double normalizeDeg(double deg) {
        double result = deg % 360.0;
        if (result <= -180.0) {
            result += 360.0;
        }
        if (result > 180.0) {
            result -= 360.0;
        }
        return result;
    }

    private static double absOrZero(double value) {
        return Double.isFinite(value) ? Math.abs(value) : 0.0;
    }

    private static double[] readDoubleArrayLenient(DataLogRecord record) {
        try {
            return record.getDoubleArray();
        } catch (Exception ignored) {
            return null;
        }
    }

    private static List<String> readStringArrayLenient(DataLogRecord record) {
        try {
            return List.of(record.getStringArray());
        } catch (Exception ignored) {
            return List.of();
        }
    }

    private static boolean readBooleanLenient(DataLogRecord record) {
        try {
            return record.getBoolean();
        } catch (Exception ignored) {
            return Boolean.parseBoolean(record.getString());
        }
    }

    private static String formatReport(List<LogReport> reports) {
        StringBuilder out = new StringBuilder();
        out.append("Heading snap mdbet analysis (qualification logs after q7)\n");
        out.append("reportPath=").append(REPORT_PATH.toAbsolutePath()).append('\n');
        out.append("csvPath=").append(CSV_PATH.toAbsolutePath()).append('\n');
        out.append('\n');

        int totalSessions = 0;
        int totalFinished = 0;
        int totalInterrupted = 0;
        int totalUnexpectedInterrupted = 0;
        int totalAnomalousSessions = 0;
        double worstFinalError = 0.0;
        double worstPostFinishError = 0.0;
        double worstTargetSpan = 0.0;
        double worstDesiredRate = 0.0;
        double worstControlGap = 0.0;

        for (LogReport report : reports) {
            totalSessions += report.sessions.size();
            for (SnapSession session : report.sessions) {
                if (session.endKind == EventKind.FINISH) {
                    totalFinished++;
                } else {
                    totalInterrupted++;
                    if (!"driverTurnInput".equals(session.termination) && !"robotDisabled".equals(session.termination)) {
                        totalUnexpectedInterrupted++;
                    }
                }
                if (!session.anomalies.isEmpty()) {
                    totalAnomalousSessions++;
                }
                if (Double.isFinite(session.finalAbsErrorDeg)) {
                    worstFinalError = Math.max(worstFinalError, session.finalAbsErrorDeg);
                }
                if (Double.isFinite(session.postFinishMaxAbsErrorDeg)) {
                    worstPostFinishError = Math.max(worstPostFinishError, session.postFinishMaxAbsErrorDeg);
                }
                if (Double.isFinite(session.targetHeadingSpanDeg)) {
                    worstTargetSpan = Math.max(worstTargetSpan, session.targetHeadingSpanDeg);
                }
                worstDesiredRate = Math.max(worstDesiredRate, session.maxAbsDesiredRateRadPerSec);
                worstControlGap = Math.max(worstControlGap, session.maxControlGapSec);
            }
        }

        out.append("overall summary\n");
        out.append(String.format(Locale.US,
                "  logs=%d sessions=%d finished=%d interrupted=%d unexpectedInterrupted=%d anomalousSessions=%d%n",
                reports.size(),
                totalSessions,
                totalFinished,
                totalInterrupted,
                totalUnexpectedInterrupted,
                totalAnomalousSessions));
        out.append(String.format(Locale.US,
                "  worstFinalAbsErrorDeg=%.2f worstPostFinishAbsErrorDeg=%.2f worstTargetSpanDeg=%.3f worstDesiredRateRadPerSec=%.6f worstControlGapSec=%.3f%n",
                worstFinalError,
                worstPostFinishError,
                worstTargetSpan,
                worstDesiredRate,
                worstControlGap));
        out.append('\n');

        for (LogReport report : reports) {
            out.append(report.matchLabel).append(' ').append(report.logPath.getFileName()).append('\n');
            out.append("  entries: ").append(report.entriesFound.describe()).append('\n');
            out.append(String.format(Locale.US,
                    "  events=%d snapRunningTransitions=%d driveDefaultTransitions=%d snapSessions=%d%n",
                    report.commandEvents.size(),
                    report.snapRunningTransitions.size(),
                    report.driveDefaultRunningTransitions.size(),
                    report.sessions.size()));
            if (report.sessions.isEmpty()) {
                out.append("  no DriveHeadingSnap sessions found\n\n");
                continue;
            }

            int unexpectedInterrupts = 0;
            for (SnapSession session : report.sessions) {
                if (session.endKind == EventKind.INTERRUPT
                        && !"driverTurnInput".equals(session.termination)
                        && !"robotDisabled".equals(session.termination)) {
                    unexpectedInterrupts++;
                }
            }
            out.append(String.format(Locale.US,
                    "  finishes=%d interrupts=%d unexpectedInterrupts=%d anomalous=%d%n",
                    report.finishCount(),
                    report.interruptCount(),
                    unexpectedInterrupts,
                    report.anomalousCount()));
            for (SnapSession session : report.sessions) {
                out.append(String.format(Locale.US,
                        "    run=%d src=%s %.3f-%.3f dur=%.3fs end=%s/%s target=%.1fdeg expected=%.1fdeg start=%.1fdeg end=%.1fdeg err[max/mean/final]=%.2f/%.2f/%.2fdeg settle[5/2]=%s/%s maxCmd=%.2f maxMeasW=%.2f lin[req/meas]=%.2f/%.2f rightX[end]=%.2f controlGap=%.3f signFlips=%d anomalies=%s%n",
                        session.runId,
                        session.source,
                        session.startSec,
                        session.endSec,
                        session.durationSec,
                        session.endKind,
                        session.termination,
                        session.targetHeadingDeg,
                        session.expectedTargetHeadingDeg,
                        session.startHeadingDeg,
                        session.endHeadingDeg,
                        session.maxAbsErrorDeg,
                        session.meanAbsErrorDeg,
                        session.finalAbsErrorDeg,
                        formatOptional(session.settleTimeTo5DegSec),
                        formatOptional(session.settleTimeTo2DegSec),
                        session.maxAbsOmegaCommandRadPerSec,
                        session.maxAbsMeasuredOmegaRadPerSec,
                        session.maxRequestedLinearSpeedMps,
                        session.maxLinearSpeedMps,
                        session.endWindowRightXAbs,
                        session.maxControlGapSec,
                        session.significantErrorSignFlips,
                        session.anomalies.isEmpty() ? "<none>" : String.join(", ", session.anomalies)));
                out.append(String.format(Locale.US,
                        "      targetSpan=%.3fdeg cardinalErr=%.3fdeg sourceErr=%.3fdeg desiredRateMax=%.6f feedbackCmdDiffMax=%.6f invalid[target/error/cmd]=%d/%d/%d lowCmd=%d wrongWay=%d measOppose=%d snapTelemetry=%s/%s driveDefault=%s/%s postFinish[err/omega]=%s/%s%n",
                        session.targetHeadingSpanDeg,
                        session.targetCardinalErrorDeg,
                        session.expectedTargetErrorDeg,
                        session.maxAbsDesiredRateRadPerSec,
                        session.maxAbsFeedbackMinusCommand,
                        session.invalidDesiredHeadingSamples,
                        session.invalidHeadingErrorSamples,
                        session.invalidOmegaCommandSamples,
                        session.largeErrorLowCommandSamples,
                        session.wrongWayCommandSamples,
                        session.measuredOmegaOpposesCommandSamples,
                        session.sawSnapRunningTrue,
                        session.sawSnapRunningFalse,
                        session.driveDefaultInterruptedNearStart,
                        session.driveDefaultRestartedNearEnd,
                        formatOptional(session.postFinishMaxAbsErrorDeg),
                        formatOptional(session.postFinishMaxAbsMeasuredOmegaRadPerSec)));
                if (!session.nearbyEndEvents.isBlank()) {
                    out.append("      nearbyEndEvents: ").append(session.nearbyEndEvents).append('\n');
                }
            }
            out.append('\n');
        }
        return out.toString();
    }

    private static String formatCsv(List<LogReport> reports) {
        StringBuilder out = new StringBuilder();
        out.append("match,log,runId,source,startSec,endSec,durationSec,endKind,termination,targetHeadingDeg,expectedTargetHeadingDeg,targetHeadingSpanDeg,targetCardinalErrorDeg,expectedTargetErrorDeg,startHeadingDeg,endHeadingDeg,maxAbsErrorDeg,meanAbsErrorDeg,finalAbsErrorDeg,settleTimeTo5DegSec,settleTimeTo2DegSec,significantErrorSignFlips,maxAbsDesiredRateRadPerSec,maxAbsFeedbackMinusCommand,maxAbsOmegaCommandRadPerSec,maxAbsMeasuredOmegaRadPerSec,maxRequestedLinearSpeedMps,maxLinearSpeedMps,maxRightXAbs,endWindowRightXAbs,maxControlGapSec,invalidDesiredHeadingSamples,invalidHeadingErrorSamples,invalidOmegaCommandSamples,largeErrorLowCommandSamples,wrongWayCommandSamples,measuredOmegaOpposesCommandSamples,sawSnapRunningTrue,sawSnapRunningFalse,driveDefaultInterruptedNearStart,driveDefaultRestartedNearEnd,postFinishMaxAbsErrorDeg,postFinishMaxAbsMeasuredOmegaRadPerSec,anomalies\n");
        for (LogReport report : reports) {
            for (SnapSession session : report.sessions) {
                out.append(csv(report.matchLabel)).append(',')
                        .append(csv(session.logName)).append(',')
                        .append(session.runId).append(',')
                        .append(csv(session.source)).append(',')
                        .append(session.startSec).append(',')
                        .append(session.endSec).append(',')
                        .append(session.durationSec).append(',')
                        .append(session.endKind).append(',')
                        .append(csv(session.termination)).append(',')
                        .append(session.targetHeadingDeg).append(',')
                        .append(session.expectedTargetHeadingDeg).append(',')
                        .append(session.targetHeadingSpanDeg).append(',')
                        .append(session.targetCardinalErrorDeg).append(',')
                        .append(session.expectedTargetErrorDeg).append(',')
                        .append(session.startHeadingDeg).append(',')
                        .append(session.endHeadingDeg).append(',')
                        .append(session.maxAbsErrorDeg).append(',')
                        .append(session.meanAbsErrorDeg).append(',')
                        .append(session.finalAbsErrorDeg).append(',')
                        .append(session.settleTimeTo5DegSec).append(',')
                        .append(session.settleTimeTo2DegSec).append(',')
                        .append(session.significantErrorSignFlips).append(',')
                        .append(session.maxAbsDesiredRateRadPerSec).append(',')
                        .append(session.maxAbsFeedbackMinusCommand).append(',')
                        .append(session.maxAbsOmegaCommandRadPerSec).append(',')
                        .append(session.maxAbsMeasuredOmegaRadPerSec).append(',')
                        .append(session.maxRequestedLinearSpeedMps).append(',')
                        .append(session.maxLinearSpeedMps).append(',')
                        .append(session.maxRightXAbs).append(',')
                        .append(session.endWindowRightXAbs).append(',')
                        .append(session.maxControlGapSec).append(',')
                        .append(session.invalidDesiredHeadingSamples).append(',')
                        .append(session.invalidHeadingErrorSamples).append(',')
                        .append(session.invalidOmegaCommandSamples).append(',')
                        .append(session.largeErrorLowCommandSamples).append(',')
                        .append(session.wrongWayCommandSamples).append(',')
                        .append(session.measuredOmegaOpposesCommandSamples).append(',')
                        .append(session.sawSnapRunningTrue).append(',')
                        .append(session.sawSnapRunningFalse).append(',')
                        .append(session.driveDefaultInterruptedNearStart).append(',')
                        .append(session.driveDefaultRestartedNearEnd).append(',')
                        .append(session.postFinishMaxAbsErrorDeg).append(',')
                        .append(session.postFinishMaxAbsMeasuredOmegaRadPerSec).append(',')
                        .append(csv(String.join(" | ", session.anomalies)))
                        .append('\n');
            }
        }
        return out.toString();
    }

    private static String csv(String value) {
        String safe = value == null ? "" : value;
        return '"' + safe.replace("\"", "\"\"") + '"';
    }

    private static String formatOptional(double value) {
        return Double.isFinite(value) ? String.format(Locale.US, "%.3f", value) : "n/a";
    }

    private static String matchLabel(Path logPath) {
        Matcher matcher = QUAL_MATCH_PATTERN.matcher(logPath.getFileName().toString());
        return matcher.matches() ? "q" + matcher.group(1) : logPath.getFileName().toString();
    }

    private record EntryInfo(String name, String type) {}

    private record RunningTransition(double timeSec, boolean running) {}

    private enum SampleKind {
        POSE,
        REQUESTED_SPEEDS,
        MEASURED_SPEEDS,
        ENABLED,
        MODE,
        MATCH_TIME,
        ALLIANCE,
        GYRO_CONNECTED,
        AXES,
        DESIRED_HEADING,
        HEADING_ERROR,
        DESIRED_RATE,
        MEASURED_OMEGA,
        FEEDBACK_OMEGA,
        OMEGA_COMMAND,
        SNAP_RUNNING,
        DRIVE_DEFAULT_RUNNING,
        RECENT_EVENTS,
        LAST_EVENT,
        OTHER
    }

    private enum EventKind {
        START,
        FINISH,
        INTERRUPT
    }

    private record CommandEvent(double timeSec, EventKind kind, int runId, String name, String source, String raw) {
        private CommandEvent withTime(double newTimeSec) {
            return new CommandEvent(newTimeSec, kind, runId, name, source, raw);
        }
    }

    private static final class CurrentState {
        private Pose2d pose = new Pose2d();
        private ChassisSpeeds requestedSpeeds = new ChassisSpeeds();
        private ChassisSpeeds measuredSpeeds = new ChassisSpeeds();
        private boolean enabled = false;
        private String mode = "";
        private double matchTime = Double.NaN;
        private String alliance = "";
        private boolean gyroConnected = true;
        private double leftX = Double.NaN;
        private double leftY = Double.NaN;
        private double rightX = Double.NaN;
        private double desiredHeadingDeg = Double.NaN;
        private double headingErrorDeg = Double.NaN;
        private double desiredRateRadPerSec = Double.NaN;
        private double measuredOmegaRadPerSec = Double.NaN;
        private double feedbackOmegaRadPerSec = Double.NaN;
        private double omegaCommandRadPerSec = Double.NaN;
        private Boolean snapRunning = null;
        private Boolean driveDefaultRunning = null;

        private StateSample snapshot(double timeSec, boolean controlUpdate, SampleKind sampleKind) {
            return new StateSample(
                    timeSec,
                    pose,
                    requestedSpeeds,
                    measuredSpeeds,
                    enabled,
                    mode,
                    matchTime,
                    alliance,
                    gyroConnected,
                    leftX,
                    leftY,
                    rightX,
                    desiredHeadingDeg,
                    headingErrorDeg,
                    desiredRateRadPerSec,
                    measuredOmegaRadPerSec,
                    feedbackOmegaRadPerSec,
                    omegaCommandRadPerSec,
                    snapRunning,
                    driveDefaultRunning,
                    controlUpdate,
                    sampleKind);
        }
    }

    private record StateSample(
            double timeSec,
            Pose2d pose,
            ChassisSpeeds requestedSpeeds,
            ChassisSpeeds measuredSpeeds,
            boolean enabled,
            String mode,
            double matchTime,
            String alliance,
            boolean gyroConnected,
            double leftX,
            double leftY,
            double rightX,
            double desiredHeadingDeg,
            double headingErrorDeg,
            double desiredRateRadPerSec,
            double measuredOmegaRadPerSec,
            double feedbackOmegaRadPerSec,
            double omegaCommandRadPerSec,
            Boolean snapRunning,
            Boolean driveDefaultRunning,
            boolean controlUpdate,
            SampleKind sampleKind) {
        private double robotHeadingDeg() {
            return pose != null ? pose.getRotation().getDegrees() : Double.NaN;
        }

        private double measuredLinearSpeed() {
            return measuredSpeeds == null ? Double.NaN : Math.hypot(measuredSpeeds.vxMetersPerSecond, measuredSpeeds.vyMetersPerSecond);
        }

        private double requestedLinearSpeed() {
            return requestedSpeeds == null ? Double.NaN : Math.hypot(requestedSpeeds.vxMetersPerSecond, requestedSpeeds.vyMetersPerSecond);
        }
    }

    private record SnapSession(
            String logName,
            int runId,
            String source,
            double startSec,
            double endSec,
            double durationSec,
            EventKind endKind,
            String termination,
            double startHeadingDeg,
            double endHeadingDeg,
            double targetHeadingDeg,
            double targetHeadingSpanDeg,
            double targetCardinalErrorDeg,
            double expectedTargetHeadingDeg,
            double expectedTargetErrorDeg,
            double maxAbsErrorDeg,
            double meanAbsErrorDeg,
            double bestAbsErrorDeg,
            double finalAbsErrorDeg,
            double settleTimeTo5DegSec,
            double settleTimeTo2DegSec,
            int significantErrorSignFlips,
            double maxAbsDesiredRateRadPerSec,
            double maxAbsFeedbackMinusCommand,
            double maxAbsOmegaCommandRadPerSec,
            double maxAbsMeasuredOmegaRadPerSec,
            double maxLinearSpeedMps,
            double maxRequestedLinearSpeedMps,
            double maxRightXAbs,
            double endWindowRightXAbs,
            double maxControlGapSec,
            int invalidDesiredHeadingSamples,
            int invalidHeadingErrorSamples,
            int invalidOmegaCommandSamples,
            int largeErrorLowCommandSamples,
            int wrongWayCommandSamples,
            int measuredOmegaOpposesCommandSamples,
            boolean sawSnapRunningTrue,
            boolean sawSnapRunningFalse,
            boolean driveDefaultInterruptedNearStart,
            boolean driveDefaultRestartedNearEnd,
            double postFinishMaxAbsErrorDeg,
            double postFinishMaxAbsMeasuredOmegaRadPerSec,
            String nearbyEndEvents,
            List<String> anomalies) {
        private SnapSession withAnomalies(List<String> newAnomalies) {
            return new SnapSession(
                    logName,
                    runId,
                    source,
                    startSec,
                    endSec,
                    durationSec,
                    endKind,
                    termination,
                    startHeadingDeg,
                    endHeadingDeg,
                    targetHeadingDeg,
                    targetHeadingSpanDeg,
                    targetCardinalErrorDeg,
                    expectedTargetHeadingDeg,
                    expectedTargetErrorDeg,
                    maxAbsErrorDeg,
                    meanAbsErrorDeg,
                    bestAbsErrorDeg,
                    finalAbsErrorDeg,
                    settleTimeTo5DegSec,
                    settleTimeTo2DegSec,
                    significantErrorSignFlips,
                    maxAbsDesiredRateRadPerSec,
                    maxAbsFeedbackMinusCommand,
                    maxAbsOmegaCommandRadPerSec,
                    maxAbsMeasuredOmegaRadPerSec,
                    maxLinearSpeedMps,
                    maxRequestedLinearSpeedMps,
                    maxRightXAbs,
                    endWindowRightXAbs,
                    maxControlGapSec,
                    invalidDesiredHeadingSamples,
                    invalidHeadingErrorSamples,
                    invalidOmegaCommandSamples,
                    largeErrorLowCommandSamples,
                    wrongWayCommandSamples,
                    measuredOmegaOpposesCommandSamples,
                    sawSnapRunningTrue,
                    sawSnapRunningFalse,
                    driveDefaultInterruptedNearStart,
                    driveDefaultRestartedNearEnd,
                    postFinishMaxAbsErrorDeg,
                    postFinishMaxAbsMeasuredOmegaRadPerSec,
                    nearbyEndEvents,
                    List.copyOf(newAnomalies));
        }
    }

    private record EntriesFound(
            boolean desiredHeading,
            boolean headingError,
            boolean desiredRate,
            boolean measuredOmega,
            boolean feedbackOmega,
            boolean omegaCommand,
            boolean recentEvents,
            boolean lastEvent,
            boolean snapRunning,
            boolean driveDefaultRunning,
            boolean pose,
            boolean requestedSpeeds,
            boolean measuredSpeeds,
            boolean axes,
            boolean enabled,
            boolean mode,
            boolean matchTime,
            boolean alliance,
            boolean gyroConnected) {
        private boolean coreEntriesPresent() {
            return desiredHeading && headingError && desiredRate && measuredOmega && feedbackOmega && omegaCommand
                    && (lastEvent || recentEvents) && driveDefaultRunning && pose && requestedSpeeds && measuredSpeeds;
        }

        private String describe() {
            return String.format(Locale.US,
                    "target=%s err=%s rate=%s measW=%s fb=%s cmd=%s events=%s snapRun=%s driveDefaultRun=%s pose=%s req=%s meas=%s axes=%s enabled=%s mode=%s matchTime=%s alliance=%s gyro=%s",
                    yesNo(desiredHeading), yesNo(headingError), yesNo(desiredRate), yesNo(measuredOmega), yesNo(feedbackOmega),
                    yesNo(omegaCommand), yesNo(lastEvent || recentEvents), yesNo(snapRunning), yesNo(driveDefaultRunning), yesNo(pose),
                    yesNo(requestedSpeeds), yesNo(measuredSpeeds), yesNo(axes), yesNo(enabled), yesNo(mode),
                    yesNo(matchTime), yesNo(alliance), yesNo(gyroConnected));
        }

        private static String yesNo(boolean value) {
            return value ? "yes" : "no";
        }
    }

    private record LogReport(
            Path logPath,
            String matchLabel,
            EntriesFound entriesFound,
            List<CommandEvent> commandEvents,
            List<RunningTransition> snapRunningTransitions,
            List<RunningTransition> driveDefaultRunningTransitions,
            List<SnapSession> sessions) {
        private int finishCount() {
            int count = 0;
            for (SnapSession session : sessions) {
                if (session.endKind == EventKind.FINISH) {
                    count++;
                }
            }
            return count;
        }

        private int interruptCount() {
            int count = 0;
            for (SnapSession session : sessions) {
                if (session.endKind == EventKind.INTERRUPT) {
                    count++;
                }
            }
            return count;
        }

        private int anomalousCount() {
            int count = 0;
            for (SnapSession session : sessions) {
                if (!session.anomalies.isEmpty()) {
                    count++;
                }
            }
            return count;
        }
    }

    private static final class EntryIds {
        private final int desiredHeadingEntry;
        private final int headingErrorEntry;
        private final int desiredRateEntry;
        private final int measuredOmegaEntry;
        private final int feedbackOmegaEntry;
        private final int omegaCommandEntry;
        private final int recentEventsEntry;
        private final int lastEventEntry;
        private final int snapRunningEntry;
        private final int driveDefaultRunningEntry;
        private final int poseEntry;
        private final int requestedSpeedsEntry;
        private final int measuredSpeedsEntry;
        private final int axesEntry;
        private final int enabledEntry;
        private final int modeEntry;
        private final int matchTimeEntry;
        private final int allianceEntry;
        private final int gyroConnectedEntry;

        private EntryIds(
                int desiredHeadingEntry,
                int headingErrorEntry,
                int desiredRateEntry,
                int measuredOmegaEntry,
                int feedbackOmegaEntry,
                int omegaCommandEntry,
                int recentEventsEntry,
                int lastEventEntry,
                int snapRunningEntry,
                int driveDefaultRunningEntry,
                int poseEntry,
                int requestedSpeedsEntry,
                int measuredSpeedsEntry,
                int axesEntry,
                int enabledEntry,
                int modeEntry,
                int matchTimeEntry,
                int allianceEntry,
                int gyroConnectedEntry) {
            this.desiredHeadingEntry = desiredHeadingEntry;
            this.headingErrorEntry = headingErrorEntry;
            this.desiredRateEntry = desiredRateEntry;
            this.measuredOmegaEntry = measuredOmegaEntry;
            this.feedbackOmegaEntry = feedbackOmegaEntry;
            this.omegaCommandEntry = omegaCommandEntry;
            this.recentEventsEntry = recentEventsEntry;
            this.lastEventEntry = lastEventEntry;
            this.snapRunningEntry = snapRunningEntry;
            this.driveDefaultRunningEntry = driveDefaultRunningEntry;
            this.poseEntry = poseEntry;
            this.requestedSpeedsEntry = requestedSpeedsEntry;
            this.measuredSpeedsEntry = measuredSpeedsEntry;
            this.axesEntry = axesEntry;
            this.enabledEntry = enabledEntry;
            this.modeEntry = modeEntry;
            this.matchTimeEntry = matchTimeEntry;
            this.allianceEntry = allianceEntry;
            this.gyroConnectedEntry = gyroConnectedEntry;
        }

        private static EntryIds from(Map<Integer, EntryInfo> entries) {
            return new EntryIds(
                    findEntry(entries, "ShotYaw/DesiredHeadingDeg"),
                    findEntry(entries, "ShotYaw/HeadingErrorDeg"),
                    findEntry(entries, "ShotYaw/DesiredHeadingRateRadPerSec"),
                    findEntry(entries, "ShotYaw/MeasuredOmegaRadPerSec"),
                    findEntry(entries, "ShotYaw/FeedbackOmegaRadPerSec"),
                    findEntry(entries, "ShotYaw/OmegaCommandRadPerSec"),
                    findEntry(entries, "Commands/recentEvents"),
                    findEntry(entries, "Commands/lastEvent"),
                    findEntry(entries, "Commands/byName/DriveHeadingSnap/running"),
                    findEntry(entries, "Commands/byName/DriveJoystickDefault/running"),
                    findEntry(entries, "Odometry/Robot"),
                    findEntry(entries, "SwerveChassisSpeeds/Requested"),
                    findEntry(entries, "SwerveChassisSpeeds/Measured"),
                    findEntry(entries, "DriverStation/Joystick0/AxisValues"),
                    findEntry(entries, "RobotState/Enabled"),
                    findEntry(entries, "RobotState/Mode"),
                    findEntry(entries, "RobotState/MatchTime"),
                    findEntry(entries, "RobotState/Alliance"),
                    findEntry(entries, "RobotState/GyroConnected"));
        }

        private EntriesFound presentSummary() {
            return new EntriesFound(
                    desiredHeadingEntry >= 0,
                    headingErrorEntry >= 0,
                    desiredRateEntry >= 0,
                    measuredOmegaEntry >= 0,
                    feedbackOmegaEntry >= 0,
                    omegaCommandEntry >= 0,
                    recentEventsEntry >= 0,
                    lastEventEntry >= 0,
                    snapRunningEntry >= 0,
                    driveDefaultRunningEntry >= 0,
                    poseEntry >= 0,
                    requestedSpeedsEntry >= 0,
                    measuredSpeedsEntry >= 0,
                    axesEntry >= 0,
                    enabledEntry >= 0,
                    modeEntry >= 0,
                    matchTimeEntry >= 0,
                    allianceEntry >= 0,
                    gyroConnectedEntry >= 0);
        }
    }

    private static int findEntry(Map<Integer, EntryInfo> entries, String suffix) {
        for (var entry : entries.entrySet()) {
            if (entry.getValue().name.endsWith(suffix)) {
                return entry.getKey();
            }
        }
        return -1;
    }
}
