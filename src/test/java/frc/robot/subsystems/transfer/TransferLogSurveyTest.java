package frc.robot.subsystems.transfer;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;

class TransferLogSurveyTest {
    private static final double COMMAND_THRESHOLD = 0.10;
    private static final double DIRECT_COMMAND_EXPECTED_PERCENT = TransferConstants.RUN_TRANSFER_PERCENT;
    private static final double EXPECTED_COMMAND_TOLERANCE = 0.02;
    private static final double LOW_VELOCITY_RPM = 250.0;
    private static final double VERY_LOW_VELOCITY_RPM = 100.0;
    private static final double EARLY_WINDOW_ALLOWANCE_SEC = 0.050;
    private static final Pattern MATCH_NUMBER_PATTERN = Pattern.compile(".*_(?:p|q)(\\d+)\\.wpilog$");
    private static final Pattern START_EVENT_PATTERN =
            Pattern.compile("^[0-9.]+s START run=(-?\\d+) name=(\\S+) source=(\\S+) requirements=(.*)$");
    private static final Pattern END_EVENT_PATTERN =
            Pattern.compile("^[0-9.]+s (FINISH|INTERRUPT) run=(-?\\d+) name=(\\S+) source=(\\S+) duration=([0-9.]+)s$");

    @Test
    void surveyTransferAcrossMdbetLogsAfterMatch7() throws IOException {
        String dirPath = System.getProperty("transferSurvey.dir");
        if (dirPath == null || dirPath.isBlank()) {
            dirPath = System.getenv("TRANSFER_SURVEY_DIR");
        }
        if (dirPath == null || dirPath.isBlank()) {
            dirPath = "logs/mdbet";
        }

        Path dir = Path.of(dirPath).toAbsolutePath();
        assumeTrue(Files.isDirectory(dir), "Transfer survey directory does not exist: " + dir);

        List<Path> logs;
        try (Stream<Path> stream = Files.list(dir)) {
            logs = stream.filter(path -> path.getFileName().toString().endsWith(".wpilog"))
                    .filter(path -> extractMatchNumber(path.getFileName().toString()) > 7)
                    .sorted()
                    .toList();
        }
        assumeTrue(!logs.isEmpty(), "No wpilogs found after match 7 in " + dir);

        List<TransferMatchSummary> summaries = new ArrayList<>();
        for (Path log : logs) {
            summaries.add(analyze(log));
        }

        String report = formatReport(dir, summaries);
        Path out = Path.of("build/reports/transfer/transfer-log-survey.txt").toAbsolutePath();
        Files.createDirectories(out.getParent());
        Files.writeString(out, report);

        System.out.println(report);
        System.out.println("Transfer survey written: " + out);

        assertTrue(report.contains("Aggregate"), "Expected aggregate section in transfer survey report.");
    }

    private static TransferMatchSummary analyze(Path wpilog) throws IOException {
        DataLogReader reader = new DataLogReader(wpilog.toString());
        if (!reader.isValid()) {
            throw new IllegalStateException("Invalid WPILOG file: " + wpilog);
        }

        EntryIds ids = new EntryIds();
        Iterator<DataLogRecord> startIterator = reader.iterator();
        while (true) {
            DataLogRecord record;
            try {
                if (!startIterator.hasNext()) {
                    break;
                }
                record = startIterator.next();
            } catch (RuntimeException exception) {
                ids.truncated = true;
                ids.parseWarnings.add("start-scan terminated early: " + exception.getMessage());
                break;
            }
            if (!record.isStart()) {
                continue;
            }
            var start = record.getStartData();
            ids.capture(start.entry, start.name, start.type);
        }

        TimelineState state = new TimelineState(wpilog.getFileName().toString());
        state.mode = "";
        state.blockReason = "";
        state.shootingState = "";
        state.lastEvent = "";

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
                state.truncated = true;
                state.parseWarnings.add("data-scan terminated early: " + exception.getMessage());
                break;
            }
            if (record.isStart() || record.isControl()) {
                continue;
            }

            long timestampUs = record.getTimestamp();
            state.advanceTo(timestampUs);

            int entry = record.getEntry();
            if (entry == ids.modeEntry) {
                state.mode = record.getString();
                state.captureModeTransition(timestampUs);
            } else if (entry == ids.enabledEntry) {
                state.enabled = record.getBoolean();
            } else if (entry == ids.matchTimeEntry) {
                state.matchTimeSec = record.getDouble();
            } else if (entry == ids.batteryVoltageEntry) {
                state.batteryVoltage = record.getDouble();
            } else if (entry == ids.commandedPercentEntry) {
                double previous = state.commandedPercent;
                state.commandedPercent = record.getDouble();
                state.nonzeroCommandedPercents.add(round3(state.commandedPercent));
                state.handleCommandedTransition(timestampUs, previous, state.commandedPercent);
            } else if (entry == ids.positionRadEntry) {
                state.positionRad = record.getDouble();
            } else if (entry == ids.velocityRpmEntry) {
                state.velocityRpm = record.getDouble();
            } else if (entry == ids.appliedVoltsEntry) {
                state.appliedVolts = record.getDouble();
            } else if (entry == ids.supplyCurrentEntry) {
                state.supplyCurrentAmps = record.getDouble();
            } else if (entry == ids.tempEntry) {
                state.tempCelsius = record.getDouble();
            } else if (entry == ids.jamCurrentCyclesEntry) {
                state.jamCurrentCycles = readIntegerLenient(record);
            } else if (entry == ids.jamDetectedEntry) {
                boolean previous = state.jamDetected;
                state.jamDetected = record.getBoolean();
                if (!previous && state.jamDetected) {
                    state.jamDetectedTimestampsSec.add(timestampUs / 1_000_000.0);
                }
            } else if (entry == ids.gateOpenEntry) {
                boolean previous = state.gateOpen;
                state.gateOpen = record.getBoolean();
                state.handleExpectedTransition(timestampUs, previousExpectedPositive(state, previous), state.expectedPositive());
            } else if (entry == ids.blockReasonEntry) {
                state.blockReason = record.getString();
            } else if (entry == ids.shootingStateEntry) {
                state.shootingState = record.getString();
            } else if (entry == ids.manualFeedOverrideEntry) {
                state.manualFeedOverride = record.getBoolean();
            } else if (entry == ids.automaticFeedEnabledEntry) {
                state.automaticFeedEnabled = record.getBoolean();
            } else if (entry == ids.shooterAtSetpointEntry) {
                state.shooterAtSetpoint = record.getBoolean();
            } else if (entry == ids.aimReadyEntry) {
                state.aimReady = record.getBoolean();
            } else if (entry == ids.lastEventEntry) {
                String event = record.getString();
                if (!event.equals(state.lastEvent)) {
                    state.lastEvent = event;
                    boolean oldExpected = state.expectedPositive();
                    state.handleCommandEvent(timestampUs, event);
                    state.handleExpectedTransition(timestampUs, oldExpected, state.expectedPositive());
                }
            }
        }

        state.finish();
        return state.toSummary(wpilog, ids);
    }

    private static boolean previousExpectedPositive(TimelineState state, boolean previousGateOpen) {
        return state.enabled && (previousGateOpen || state.directPositiveActiveCount > 0);
    }

    private static int extractMatchNumber(String name) {
        Matcher matcher = MATCH_NUMBER_PATTERN.matcher(name);
        if (!matcher.matches()) {
            return Integer.MIN_VALUE;
        }
        return Integer.parseInt(matcher.group(1));
    }

    private static int readIntegerLenient(DataLogRecord record) {
        try {
            return (int) record.getInteger();
        } catch (RuntimeException ignored) {
            return (int) Math.round(record.getDouble());
        }
    }

    private static boolean requiresTransfer(String requirements) {
        return requirements != null && requirements.contains("transfer");
    }

    private static boolean isDirectPositiveFeedCommand(String commandName) {
        if (commandName == null || commandName.isBlank()) {
            return false;
        }
        if ("ShootCoordinatorManualFeed".equals(commandName)) {
            return true;
        }
        return commandName.contains("TransferRun") || commandName.contains("TransferDashboardTune");
    }

    private static double round3(double value) {
        return Math.round(value * 1000.0) / 1000.0;
    }

    private static String formatReport(Path dir, List<TransferMatchSummary> summaries) {
        AggregateSummary aggregate = AggregateSummary.from(summaries);
        StringBuilder out = new StringBuilder();
        out.append("Transfer log survey").append(System.lineSeparator());
        out.append("dir=").append(dir).append(System.lineSeparator());
        out.append("matchesAnalyzed=").append(summaries.size()).append(System.lineSeparator());
        out.append(System.lineSeparator());

        for (TransferMatchSummary summary : summaries) {
            out.append(formatMatchSummary(summary)).append(System.lineSeparator());
        }

        out.append("Aggregate").append(System.lineSeparator());
        out.append(String.format(Locale.US, "  totalEnabledSec=%.2f autoSec=%.2f teleopSec=%.2f%n",
                aggregate.totalEnabledSec, aggregate.totalAutoSec, aggregate.totalTeleopSec));
        out.append(String.format(Locale.US,
                "  expectedFeedSec=%.2f alignedPositiveSec=%.2f missingExpectedCommandSec=%.3f unexplainedPositiveSec=%.3f disabledPositiveSec=%.3f%n",
                aggregate.expectedFeedSec,
                aggregate.alignedPositiveSec,
                aggregate.missingExpectedCommandSec,
                aggregate.unexplainedPositiveSec,
                aggregate.disabledPositiveSec));
        out.append(String.format(Locale.US,
                "  positiveSegments=%d totalPositiveSec=%.2f totalReverseSec=%.2f%n",
                aggregate.totalPositiveSegments,
                aggregate.totalPositiveSec,
                aggregate.totalReverseSec));
        out.append(String.format(Locale.US,
                "  positiveCurrent %s%n",
                aggregate.positiveCurrentStats.describe()));
        out.append(String.format(Locale.US,
                "  positiveAbsVelocity %s%n",
                aggregate.positiveVelocityStats.describe()));
        out.append(String.format(Locale.US,
                "  positiveAppliedVolts %s%n",
                aggregate.positiveAppliedVoltsStats.describe()));
        out.append(String.format(Locale.US,
                "  positiveBatteryVoltage %s%n",
                aggregate.positiveBatteryStats.describe()));
        out.append(String.format(Locale.US,
                "  positiveTempCelsius %s%n",
                aggregate.positiveTempStats.describe()));
        out.append(String.format(Locale.US,
                "  highCurrentWhilePositiveSec=%.3f lowVelocityWhilePositiveSec=%.3f veryLowVelocityWhilePositiveSec=%.3f jamDetectedCount=%d jamDetectedSec=%.3f%n",
                aggregate.highCurrentWhilePositiveSec,
                aggregate.lowVelocityWhilePositiveSec,
                aggregate.veryLowVelocityWhilePositiveSec,
                aggregate.jamDetectedCount,
                aggregate.jamDetectedSec));
        out.append(String.format(Locale.US,
                "  feedWindows=%d cleanWindows=%d windowsWithMissingAfterSpinup=%d worstWindowMissingAfterSpinupSec=%.3f maxStartLatencySec=%.3f%n",
                aggregate.feedWindowCount,
                aggregate.cleanFeedWindows,
                aggregate.feedWindowsWithMissingAfterSpinup,
                aggregate.worstFeedWindowMissingAfterSpinupSec,
                aggregate.maxFeedWindowStartLatencySec));
        out.append("  aggregateTransferCommands").append(System.lineSeparator());
        for (CommandStats stats : aggregate.commandStats.values()) {
            out.append(String.format(Locale.US,
                    "    %s starts=%d finishes=%d interrupts=%d total=%.2fs avg=%.3fs max=%.3fs transferReq=%s directPositive=%s%n",
                    stats.name,
                    stats.starts,
                    stats.finishes,
                    stats.interrupts,
                    stats.totalDurationSec,
                    stats.averageDurationSec(),
                    stats.maxDurationSec,
                    stats.transferRequirement,
                    stats.directPositive));
        }
        out.append("  notableWorstFeedWindows").append(System.lineSeparator());
        for (FeedWindowSummary window : aggregate.worstFeedWindows()) {
            out.append("    ").append(window.describe()).append(System.lineSeparator());
        }
        out.append("  notableWorstPositiveSegments").append(System.lineSeparator());
        for (TransferSegmentSummary segment : aggregate.worstPositiveSegments()) {
            out.append("    ").append(segment.describe()).append(System.lineSeparator());
        }
        out.append("  verdict").append(System.lineSeparator());
        out.append("    ").append(aggregate.verdict()).append(System.lineSeparator());
        return out.toString();
    }

    private static String formatMatchSummary(TransferMatchSummary summary) {
        StringBuilder out = new StringBuilder();
        out.append(summary.fileName).append(System.lineSeparator());
        out.append(String.format(Locale.US,
                "  enabledSec=%.2f autoSec=%.2f teleopSec=%.2f truncated=%s%n",
                summary.enabledSec,
                summary.autoSec,
                summary.teleopSec,
                summary.truncated));
        out.append(String.format(Locale.US,
                "  commandedValues=%s%n",
                summary.nonzeroCommandedPercents));
        out.append(String.format(Locale.US,
                "  expectedFeedSec=%.2f alignedPositiveSec=%.2f missingExpectedCommandSec=%.3f unexplainedPositiveSec=%.3f disabledPositiveSec=%.3f%n",
                summary.expectedFeedSec,
                summary.alignedPositiveSec,
                summary.missingExpectedCommandSec,
                summary.unexplainedPositiveSec,
                summary.disabledPositiveSec));
        out.append(String.format(Locale.US,
                "  positiveSegments=%d totalPositiveSec=%.2f totalReverseSec=%.2f%n",
                summary.positiveSegments.size(),
                summary.totalPositiveSec,
                summary.totalReverseSec));
        out.append(String.format(Locale.US,
                "  positiveCurrent %s%n",
                summary.positiveCurrentStats.describe()));
        out.append(String.format(Locale.US,
                "  positiveAbsVelocity %s%n",
                summary.positiveVelocityStats.describe()));
        out.append(String.format(Locale.US,
                "  positiveAppliedVolts %s%n",
                summary.positiveAppliedVoltsStats.describe()));
        out.append(String.format(Locale.US,
                "  positiveBatteryVoltage %s%n",
                summary.positiveBatteryStats.describe()));
        out.append(String.format(Locale.US,
                "  positiveTempCelsius %s%n",
                summary.positiveTempStats.describe()));
        out.append(String.format(Locale.US,
                "  highCurrentWhilePositiveSec=%.3f lowVelocityWhilePositiveSec=%.3f veryLowVelocityWhilePositiveSec=%.3f jamDetectedCount=%d jamDetectedSec=%.3f%n",
                summary.highCurrentWhilePositiveSec,
                summary.lowVelocityWhilePositiveSec,
                summary.veryLowVelocityWhilePositiveSec,
                summary.jamDetectedCount,
                summary.jamDetectedSec));
        out.append("  transferCommands").append(System.lineSeparator());
        for (CommandStats stats : summary.commandStats.values()) {
            if (!stats.transferRequirement) {
                continue;
            }
            out.append(String.format(Locale.US,
                    "    %s starts=%d finishes=%d interrupts=%d total=%.2fs avg=%.3fs max=%.3fs directPositive=%s short<0.25s=%d%n",
                    stats.name,
                    stats.starts,
                    stats.finishes,
                    stats.interrupts,
                    stats.totalDurationSec,
                    stats.averageDurationSec(),
                    stats.maxDurationSec,
                    stats.directPositive,
                    stats.shortRunsUnderQuarterSecond));
        }
        out.append("  feedWindows").append(System.lineSeparator());
        if (summary.feedWindows.isEmpty()) {
            out.append("    none").append(System.lineSeparator());
        } else {
            for (FeedWindowSummary window : summary.feedWindows) {
                out.append("    ").append(window.describe()).append(System.lineSeparator());
            }
        }
        out.append("  positiveSegmentsDetail").append(System.lineSeparator());
        if (summary.positiveSegments.isEmpty()) {
            out.append("    none").append(System.lineSeparator());
        } else {
            for (TransferSegmentSummary segment : summary.positiveSegments) {
                out.append("    ").append(segment.describe()).append(System.lineSeparator());
            }
        }
        if (!summary.jamDetectedTimestampsSec.isEmpty()) {
            out.append("  jamDetectedTimestampsSec=").append(summary.jamDetectedTimestampsSec).append(System.lineSeparator());
        }
        if (!summary.parseWarnings.isEmpty()) {
            out.append("  parseWarnings").append(System.lineSeparator());
            for (String warning : summary.parseWarnings) {
                out.append("    ").append(warning).append(System.lineSeparator());
            }
        }
        return out.toString();
    }

    private static final class TimelineState {
        private final String fileName;
        private final List<String> parseWarnings = new ArrayList<>();
        private final List<FeedWindowSummary> feedWindows = new ArrayList<>();
        private final List<TransferSegmentSummary> positiveSegments = new ArrayList<>();
        private final List<Double> jamDetectedTimestampsSec = new ArrayList<>();
        private final Map<String, CommandStats> commandStats = new LinkedHashMap<>();
        private final Map<Integer, ActiveCommand> activeTransferCommands = new HashMap<>();
        private final Set<Double> nonzeroCommandedPercents = new LinkedHashSet<>();
        private final WeightedStats positiveCurrentStats = new WeightedStats();
        private final WeightedStats positiveVelocityStats = new WeightedStats();
        private final WeightedStats positiveAppliedVoltsStats = new WeightedStats();
        private final WeightedStats positiveBatteryStats = new WeightedStats();
        private final WeightedStats positiveTempStats = new WeightedStats();
        private long previousTimestampUs = -1L;
        private boolean truncated = false;

        private boolean enabled = false;
        private String mode = "";
        private double matchTimeSec = Double.NaN;
        private double batteryVoltage = Double.NaN;
        private double commandedPercent = 0.0;
        private double positionRad = Double.NaN;
        private double velocityRpm = Double.NaN;
        private double appliedVolts = Double.NaN;
        private double supplyCurrentAmps = Double.NaN;
        private double tempCelsius = Double.NaN;
        private int jamCurrentCycles = 0;
        private boolean jamDetected = false;
        private boolean gateOpen = false;
        private String blockReason = "";
        private String shootingState = "";
        private boolean manualFeedOverride = false;
        private boolean automaticFeedEnabled = true;
        private boolean shooterAtSetpoint = false;
        private boolean aimReady = false;
        private String lastEvent = "";
        private int directPositiveActiveCount = 0;

        private double enabledSec = 0.0;
        private double autoSec = 0.0;
        private double teleopSec = 0.0;
        private double expectedFeedSec = 0.0;
        private double alignedPositiveSec = 0.0;
        private double missingExpectedCommandSec = 0.0;
        private double unexplainedPositiveSec = 0.0;
        private double disabledPositiveSec = 0.0;
        private double totalPositiveSec = 0.0;
        private double totalReverseSec = 0.0;
        private double highCurrentWhilePositiveSec = 0.0;
        private double lowVelocityWhilePositiveSec = 0.0;
        private double veryLowVelocityWhilePositiveSec = 0.0;
        private double jamDetectedSec = 0.0;
        private int jamDetectedCount = 0;

        private FeedWindowBuilder activeFeedWindow = null;
        private TransferSegmentBuilder activePositiveSegment = null;

        private TimelineState(String fileName) {
            this.fileName = fileName;
        }

        private void advanceTo(long timestampUs) {
            if (previousTimestampUs < 0L) {
                previousTimestampUs = timestampUs;
                return;
            }
            long deltaUs = Math.max(0L, timestampUs - previousTimestampUs);
            double deltaSec = deltaUs / 1_000_000.0;
            if (deltaSec <= 0.0) {
                previousTimestampUs = timestampUs;
                return;
            }

            if (enabled) {
                enabledSec += deltaSec;
                if ("AUTO".equals(mode)) {
                    autoSec += deltaSec;
                } else if ("TELEOP".equals(mode)) {
                    teleopSec += deltaSec;
                }
            }

            boolean positive = commandedPositive();
            boolean reverse = commandedReverse();
            boolean expected = expectedPositive();

            if (expected) {
                expectedFeedSec += deltaSec;
            }
            if (positive) {
                totalPositiveSec += deltaSec;
                positiveCurrentStats.add(supplyCurrentAmps, deltaSec);
                positiveVelocityStats.add(Math.abs(velocityRpm), deltaSec);
                positiveAppliedVoltsStats.add(appliedVolts, deltaSec);
                positiveBatteryStats.add(batteryVoltage, deltaSec);
                positiveTempStats.add(tempCelsius, deltaSec);
                if (Math.abs(supplyCurrentAmps) >= TransferConstants.JAM_CURRENT_THRESHOLD_AMPS) {
                    highCurrentWhilePositiveSec += deltaSec;
                }
                if (Math.abs(velocityRpm) < LOW_VELOCITY_RPM) {
                    lowVelocityWhilePositiveSec += deltaSec;
                }
                if (Math.abs(velocityRpm) < VERY_LOW_VELOCITY_RPM) {
                    veryLowVelocityWhilePositiveSec += deltaSec;
                }
            }
            if (reverse) {
                totalReverseSec += deltaSec;
            }
            if (expected && positive) {
                alignedPositiveSec += deltaSec;
            }
            if (expected && !positive) {
                missingExpectedCommandSec += deltaSec;
            }
            if (!expected && positive) {
                unexplainedPositiveSec += deltaSec;
            }
            if (!enabled && (positive || reverse)) {
                disabledPositiveSec += deltaSec;
            }
            if (jamDetected) {
                jamDetectedSec += deltaSec;
            }

            if (activeFeedWindow != null) {
                activeFeedWindow.totalSec += deltaSec;
                if (positive) {
                    activeFeedWindow.positiveSec += deltaSec;
                } else {
                    activeFeedWindow.missingSec += deltaSec;
                    if (activeFeedWindow.firstPositiveTimestampUs >= 0L) {
                        activeFeedWindow.missingAfterFirstPositiveSec += deltaSec;
                    }
                }
            }
            if (activePositiveSegment != null) {
                activePositiveSegment.durationSec += deltaSec;
                activePositiveSegment.currentStats.add(supplyCurrentAmps, deltaSec);
                activePositiveSegment.absVelocityStats.add(Math.abs(velocityRpm), deltaSec);
                activePositiveSegment.appliedVoltsStats.add(appliedVolts, deltaSec);
                activePositiveSegment.batteryStats.add(batteryVoltage, deltaSec);
                activePositiveSegment.maxJamCurrentCycles = Math.max(activePositiveSegment.maxJamCurrentCycles, jamCurrentCycles);
                activePositiveSegment.jamDetected |= jamDetected;
                activePositiveSegment.maxAbsCurrentAmps = Math.max(activePositiveSegment.maxAbsCurrentAmps, Math.abs(supplyCurrentAmps));
                activePositiveSegment.minAbsVelocityRpm = Math.min(activePositiveSegment.minAbsVelocityRpm, Math.abs(velocityRpm));
                activePositiveSegment.maxAbsVelocityRpm = Math.max(activePositiveSegment.maxAbsVelocityRpm, Math.abs(velocityRpm));
                if (Double.isFinite(positionRad)) {
                    activePositiveSegment.lastKnownPositionRad = positionRad;
                }
            }

            previousTimestampUs = timestampUs;
        }

        private void captureModeTransition(long timestampUs) {
            boolean oldExpected = activeFeedWindow != null;
            boolean newExpected = expectedPositive();
            if (oldExpected != newExpected) {
                handleExpectedTransition(timestampUs, oldExpected, newExpected);
            }
        }

        private void handleCommandedTransition(long timestampUs, double previousCommandedPercent, double newCommandedPercent) {
            boolean wasPositive = previousCommandedPercent > COMMAND_THRESHOLD;
            boolean isPositive = newCommandedPercent > COMMAND_THRESHOLD;
            boolean wasReverse = previousCommandedPercent < -COMMAND_THRESHOLD;
            boolean isReverse = newCommandedPercent < -COMMAND_THRESHOLD;

            if (!wasPositive && isPositive) {
                openPositiveSegment(timestampUs);
                if (activeFeedWindow != null && activeFeedWindow.firstPositiveTimestampUs < 0L) {
                    activeFeedWindow.firstPositiveTimestampUs = timestampUs;
                }
                if (activeFeedWindow != null && activeFeedWindow.openGapTimestampUs >= 0L) {
                    activeFeedWindow.closeGap(timestampUs);
                }
            } else if (wasPositive && !isPositive) {
                closePositiveSegment(timestampUs);
                if (activeFeedWindow != null) {
                    activeFeedWindow.openGap(timestampUs);
                }
            }

            if (!wasReverse && isReverse) {
                closePositiveSegment(timestampUs);
            }
            if (wasReverse && !isReverse && activeFeedWindow != null && !commandedPositive()) {
                activeFeedWindow.openGap(timestampUs);
            }
        }

        private void handleExpectedTransition(long timestampUs, boolean wasExpected, boolean isExpected) {
            if (!wasExpected && isExpected) {
                activeFeedWindow = new FeedWindowBuilder(fileName, feedWindows.size() + 1, timestampUs, mode, matchTimeSec,
                        expectationReason());
                if (commandedPositive()) {
                    activeFeedWindow.firstPositiveTimestampUs = timestampUs;
                } else {
                    activeFeedWindow.openGap(timestampUs);
                }
            } else if (wasExpected && !isExpected) {
                if (activeFeedWindow != null) {
                    activeFeedWindow.close(timestampUs);
                    feedWindows.add(activeFeedWindow.toSummary());
                    activeFeedWindow = null;
                }
            }
        }

        private void openPositiveSegment(long timestampUs) {
            if (activePositiveSegment != null) {
                return;
            }
            activePositiveSegment = new TransferSegmentBuilder(fileName, positiveSegments.size() + 1, timestampUs, mode,
                    matchTimeSec, expectationReason(), gateOpen, batteryVoltage, positionRad);
        }

        private void closePositiveSegment(long timestampUs) {
            if (activePositiveSegment == null) {
                return;
            }
            activePositiveSegment.close(timestampUs, matchTimeSec, positionRad);
            positiveSegments.add(activePositiveSegment.toSummary());
            activePositiveSegment = null;
        }

        private void handleCommandEvent(long timestampUs, String event) {
            Matcher startMatcher = START_EVENT_PATTERN.matcher(event);
            if (startMatcher.matches()) {
                int runId = Integer.parseInt(startMatcher.group(1));
                String name = startMatcher.group(2);
                String source = startMatcher.group(3);
                String requirements = startMatcher.group(4);
                boolean transferRequirement = requiresTransfer(requirements);
                CommandStats stats = commandStats.computeIfAbsent(name, key -> new CommandStats(name));
                stats.transferRequirement |= transferRequirement;
                stats.directPositive |= isDirectPositiveFeedCommand(name);
                stats.starts++;
                if (transferRequirement) {
                    ActiveCommand active = new ActiveCommand(runId, name, source, requirements, timestampUs);
                    activeTransferCommands.put(runId, active);
                    if (isDirectPositiveFeedCommand(name)) {
                        directPositiveActiveCount++;
                    }
                }
                return;
            }

            Matcher endMatcher = END_EVENT_PATTERN.matcher(event);
            if (endMatcher.matches()) {
                boolean interrupted = "INTERRUPT".equals(endMatcher.group(1));
                int runId = Integer.parseInt(endMatcher.group(2));
                String name = endMatcher.group(3);
                double durationSec = Double.parseDouble(endMatcher.group(5));
                CommandStats stats = commandStats.computeIfAbsent(name, key -> new CommandStats(name));
                if (interrupted) {
                    stats.interrupts++;
                } else {
                    stats.finishes++;
                }
                stats.totalDurationSec += durationSec;
                stats.maxDurationSec = Math.max(stats.maxDurationSec, durationSec);
                if (durationSec < 0.25) {
                    stats.shortRunsUnderQuarterSecond++;
                }
                ActiveCommand active = activeTransferCommands.remove(runId);
                if (active != null) {
                    stats.transferRequirement = true;
                    stats.directPositive |= active.directPositive;
                    if (active.directPositive) {
                        directPositiveActiveCount = Math.max(0, directPositiveActiveCount - 1);
                    }
                }
            }
        }

        private boolean expectedPositive() {
            return enabled && (gateOpen || directPositiveActiveCount > 0);
        }

        private boolean commandedPositive() {
            return commandedPercent > COMMAND_THRESHOLD;
        }

        private boolean commandedReverse() {
            return commandedPercent < -COMMAND_THRESHOLD;
        }

        private String expectationReason() {
            List<String> reasons = new ArrayList<>();
            if (gateOpen) {
                reasons.add("GateOpen");
            }
            if (directPositiveActiveCount > 0) {
                List<String> directNames = new ArrayList<>();
                for (ActiveCommand active : activeTransferCommands.values()) {
                    if (active.directPositive) {
                        directNames.add(active.name);
                    }
                }
                Collections.sort(directNames);
                reasons.add("Direct=" + directNames);
            }
            if (reasons.isEmpty()) {
                return "None";
            }
            return String.join("+", reasons);
        }

        private void finish() {
            if (previousTimestampUs >= 0L) {
                if (activeFeedWindow != null) {
                    activeFeedWindow.close(previousTimestampUs);
                    feedWindows.add(activeFeedWindow.toSummary());
                    activeFeedWindow = null;
                }
                closePositiveSegment(previousTimestampUs);
            }
            jamDetectedCount = jamDetectedTimestampsSec.size();
        }

        private TransferMatchSummary toSummary(Path wpilog, EntryIds ids) {
            return new TransferMatchSummary(
                    wpilog.toAbsolutePath().toString(),
                    fileName,
                    truncated || ids.truncated,
                    combineWarnings(parseWarnings, ids.parseWarnings),
                    new ArrayList<>(nonzeroCommandedPercents),
                    enabledSec,
                    autoSec,
                    teleopSec,
                    expectedFeedSec,
                    alignedPositiveSec,
                    missingExpectedCommandSec,
                    unexplainedPositiveSec,
                    disabledPositiveSec,
                    totalPositiveSec,
                    totalReverseSec,
                    highCurrentWhilePositiveSec,
                    lowVelocityWhilePositiveSec,
                    veryLowVelocityWhilePositiveSec,
                    jamDetectedSec,
                    jamDetectedCount,
                    positiveCurrentStats.copy(),
                    positiveVelocityStats.copy(),
                    positiveAppliedVoltsStats.copy(),
                    positiveBatteryStats.copy(),
                    positiveTempStats.copy(),
                    new ArrayList<>(feedWindows),
                    new ArrayList<>(positiveSegments),
                    cloneCommandStats(commandStats),
                    new ArrayList<>(jamDetectedTimestampsSec));
        }

        private static List<String> combineWarnings(List<String> left, List<String> right) {
            List<String> combined = new ArrayList<>(left.size() + right.size());
            combined.addAll(left);
            combined.addAll(right);
            return combined;
        }

        private static Map<String, CommandStats> cloneCommandStats(Map<String, CommandStats> source) {
            Map<String, CommandStats> copy = new LinkedHashMap<>();
            for (Map.Entry<String, CommandStats> entry : source.entrySet()) {
                copy.put(entry.getKey(), entry.getValue().copy());
            }
            return copy;
        }
    }

    private static final class FeedWindowBuilder {
        private final String fileName;
        private final int index;
        private final long startTimestampUs;
        private final String startMode;
        private final double startMatchTimeSec;
        private final String reason;
        private double totalSec = 0.0;
        private double positiveSec = 0.0;
        private double missingSec = 0.0;
        private double missingAfterFirstPositiveSec = 0.0;
        private long firstPositiveTimestampUs = -1L;
        private long openGapTimestampUs = -1L;
        private double maxGapSec = 0.0;
        private double maxGapAfterFirstPositiveSec = 0.0;
        private long endTimestampUs = -1L;

        private FeedWindowBuilder(String fileName, int index, long startTimestampUs, String startMode, double startMatchTimeSec,
                String reason) {
            this.fileName = fileName;
            this.index = index;
            this.startTimestampUs = startTimestampUs;
            this.startMode = startMode;
            this.startMatchTimeSec = startMatchTimeSec;
            this.reason = reason;
        }

        private void openGap(long timestampUs) {
            if (openGapTimestampUs < 0L) {
                openGapTimestampUs = timestampUs;
            }
        }

        private void closeGap(long timestampUs) {
            if (openGapTimestampUs < 0L) {
                return;
            }
            double gapSec = Math.max(0.0, (timestampUs - openGapTimestampUs) / 1_000_000.0);
            maxGapSec = Math.max(maxGapSec, gapSec);
            if (firstPositiveTimestampUs >= 0L && openGapTimestampUs >= firstPositiveTimestampUs) {
                maxGapAfterFirstPositiveSec = Math.max(maxGapAfterFirstPositiveSec, gapSec);
            }
            openGapTimestampUs = -1L;
        }

        private void close(long timestampUs) {
            closeGap(timestampUs);
            endTimestampUs = timestampUs;
        }

        private FeedWindowSummary toSummary() {
            double durationSec = Math.max(0.0, (endTimestampUs - startTimestampUs) / 1_000_000.0);
            double startLatencySec = firstPositiveTimestampUs < 0L
                    ? durationSec
                    : Math.max(0.0, (firstPositiveTimestampUs - startTimestampUs) / 1_000_000.0);
            return new FeedWindowSummary(
                    fileName,
                    index,
                    startTimestampUs / 1_000_000.0,
                    durationSec,
                    startMode,
                    startMatchTimeSec,
                    reason,
                    totalSec,
                    positiveSec,
                    missingSec,
                    missingAfterFirstPositiveSec,
                    maxGapSec,
                    maxGapAfterFirstPositiveSec,
                    startLatencySec);
        }
    }

    private static final class TransferSegmentBuilder {
        private final String fileName;
        private final int index;
        private final long startTimestampUs;
        private final String startMode;
        private final double startMatchTimeSec;
        private final String reason;
        private final boolean gateOpenAtStart;
        private final double startBatteryVoltage;
        private final double startPositionRad;
        private final WeightedStats currentStats = new WeightedStats();
        private final WeightedStats absVelocityStats = new WeightedStats();
        private final WeightedStats appliedVoltsStats = new WeightedStats();
        private final WeightedStats batteryStats = new WeightedStats();
        private double durationSec = 0.0;
        private double maxAbsCurrentAmps = 0.0;
        private double minAbsVelocityRpm = Double.POSITIVE_INFINITY;
        private double maxAbsVelocityRpm = 0.0;
        private int maxJamCurrentCycles = 0;
        private boolean jamDetected = false;
        private double endMatchTimeSec = Double.NaN;
        private double endPositionRad = Double.NaN;
        private double lastKnownPositionRad = Double.NaN;

        private TransferSegmentBuilder(String fileName, int index, long startTimestampUs, String startMode, double startMatchTimeSec,
                String reason, boolean gateOpenAtStart, double startBatteryVoltage, double startPositionRad) {
            this.fileName = fileName;
            this.index = index;
            this.startTimestampUs = startTimestampUs;
            this.startMode = startMode;
            this.startMatchTimeSec = startMatchTimeSec;
            this.reason = reason;
            this.gateOpenAtStart = gateOpenAtStart;
            this.startBatteryVoltage = startBatteryVoltage;
            this.startPositionRad = startPositionRad;
            this.lastKnownPositionRad = startPositionRad;
        }

        private void close(long timestampUs, double endMatchTimeSec, double currentPositionRad) {
            this.endMatchTimeSec = endMatchTimeSec;
            this.endPositionRad = Double.isFinite(currentPositionRad) ? currentPositionRad : lastKnownPositionRad;
        }

        private TransferSegmentSummary toSummary() {
            double positionDeltaRot = Double.NaN;
            if (Double.isFinite(startPositionRad) && Double.isFinite(endPositionRad)) {
                positionDeltaRot = (endPositionRad - startPositionRad) / (2.0 * Math.PI);
            }
            return new TransferSegmentSummary(
                    fileName,
                    index,
                    startTimestampUs / 1_000_000.0,
                    durationSec,
                    startMode,
                    startMatchTimeSec,
                    endMatchTimeSec,
                    reason,
                    gateOpenAtStart,
                    startBatteryVoltage,
                    positionDeltaRot,
                    currentStats.copy(),
                    absVelocityStats.copy(),
                    appliedVoltsStats.copy(),
                    batteryStats.copy(),
                    maxAbsCurrentAmps,
                    minAbsVelocityRpm,
                    maxAbsVelocityRpm,
                    maxJamCurrentCycles,
                    jamDetected);
        }
    }

    private static final class EntryIds {
        private final Map<Integer, EntryInfo> entries = new HashMap<>();
        private final List<String> parseWarnings = new ArrayList<>();
        private boolean truncated = false;
        private int modeEntry = -1;
        private int enabledEntry = -1;
        private int matchTimeEntry = -1;
        private int batteryVoltageEntry = -1;
        private int commandedPercentEntry = -1;
        private int positionRadEntry = -1;
        private int velocityRpmEntry = -1;
        private int appliedVoltsEntry = -1;
        private int supplyCurrentEntry = -1;
        private int tempEntry = -1;
        private int jamCurrentCyclesEntry = -1;
        private int jamDetectedEntry = -1;
        private int gateOpenEntry = -1;
        private int blockReasonEntry = -1;
        private int shootingStateEntry = -1;
        private int manualFeedOverrideEntry = -1;
        private int automaticFeedEnabledEntry = -1;
        private int shooterAtSetpointEntry = -1;
        private int aimReadyEntry = -1;
        private int lastEventEntry = -1;

        private void capture(int entry, String name, String type) {
            entries.put(entry, new EntryInfo(name, type));
            String normalized = normalizeEntryName(name);
            if ("RobotState/Mode".equals(normalized)) {
                modeEntry = entry;
            } else if ("RobotState/Enabled".equals(normalized)) {
                enabledEntry = entry;
            } else if ("RobotState/MatchTime".equals(normalized)) {
                matchTimeEntry = entry;
            } else if ("RobotState/BatteryVoltage".equals(normalized) || "PowerDistribution/totalVoltageVolts".equals(normalized)) {
                batteryVoltageEntry = entry;
            } else if ("Transfer/CommandedPercent".equals(normalized)) {
                commandedPercentEntry = entry;
            } else if ("Transfer/PositionRad".equals(normalized)) {
                positionRadEntry = entry;
            } else if ("Transfer/VelocityRpm".equals(normalized)) {
                velocityRpmEntry = entry;
            } else if ("Transfer/AppliedVolts".equals(normalized)) {
                appliedVoltsEntry = entry;
            } else if ("Transfer/SupplyCurrentAmps".equals(normalized)) {
                supplyCurrentEntry = entry;
            } else if ("Transfer/TempCelsius".equals(normalized)) {
                tempEntry = entry;
            } else if ("Transfer/JamCurrentCycles".equals(normalized)) {
                jamCurrentCyclesEntry = entry;
            } else if ("Transfer/JamDetected".equals(normalized)) {
                jamDetectedEntry = entry;
            } else if ("Shooting/GateOpen".equals(normalized)) {
                gateOpenEntry = entry;
            } else if ("Shooting/BlockReason".equals(normalized)) {
                blockReasonEntry = entry;
            } else if ("Shooting/State".equals(normalized)) {
                shootingStateEntry = entry;
            } else if ("Shooting/ManualFeedOverride".equals(normalized)) {
                manualFeedOverrideEntry = entry;
            } else if ("Shooting/AutomaticFeedEnabled".equals(normalized)) {
                automaticFeedEnabledEntry = entry;
            } else if ("Shooting/ShooterAtSetpoint".equals(normalized)) {
                shooterAtSetpointEntry = entry;
            } else if ("Shooting/AimReady".equals(normalized)) {
                aimReadyEntry = entry;
            } else if ("Commands/lastEvent".equals(normalized)) {
                lastEventEntry = entry;
            }
        }
    }

    private static String normalizeEntryName(String name) {
        if (name == null) {
            return "";
        }
        String normalized = name.startsWith("/") ? name.substring(1) : name;
        if (normalized.startsWith("RealOutputs/")) {
            normalized = normalized.substring("RealOutputs/".length());
        }
        return normalized;
    }

    private record EntryInfo(String name, String type) {}

    private static final class ActiveCommand {
        private final int runId;
        private final String name;
        private final String source;
        private final String requirements;
        private final long startTimestampUs;
        private final boolean directPositive;

        private ActiveCommand(int runId, String name, String source, String requirements, long startTimestampUs) {
            this.runId = runId;
            this.name = name;
            this.source = source;
            this.requirements = requirements;
            this.startTimestampUs = startTimestampUs;
            this.directPositive = isDirectPositiveFeedCommand(name);
        }
    }

    private static final class WeightedStats {
        private double totalWeight = 0.0;
        private double weightedSum = 0.0;
        private double min = Double.POSITIVE_INFINITY;
        private double max = Double.NEGATIVE_INFINITY;

        private void add(double value, double weight) {
            if (!Double.isFinite(value) || weight <= 0.0) {
                return;
            }
            totalWeight += weight;
            weightedSum += value * weight;
            min = Math.min(min, value);
            max = Math.max(max, value);
        }

        private double mean() {
            return totalWeight <= 0.0 ? Double.NaN : weightedSum / totalWeight;
        }

        private String describe() {
            if (totalWeight <= 0.0) {
                return "[]";
            }
            return String.format(Locale.US, "min=%.2f mean=%.2f max=%.2f sampleSec=%.2f", min, mean(), max, totalWeight);
        }

        private WeightedStats copy() {
            WeightedStats copy = new WeightedStats();
            copy.totalWeight = totalWeight;
            copy.weightedSum = weightedSum;
            copy.min = min;
            copy.max = max;
            return copy;
        }
    }

    private static final class CommandStats {
        private final String name;
        private boolean transferRequirement = false;
        private boolean directPositive = false;
        private int starts = 0;
        private int finishes = 0;
        private int interrupts = 0;
        private int shortRunsUnderQuarterSecond = 0;
        private double totalDurationSec = 0.0;
        private double maxDurationSec = 0.0;

        private CommandStats(String name) {
            this.name = name;
        }

        private double averageDurationSec() {
            int ended = finishes + interrupts;
            return ended == 0 ? Double.NaN : totalDurationSec / ended;
        }

        private CommandStats copy() {
            CommandStats copy = new CommandStats(name);
            copy.transferRequirement = transferRequirement;
            copy.directPositive = directPositive;
            copy.starts = starts;
            copy.finishes = finishes;
            copy.interrupts = interrupts;
            copy.shortRunsUnderQuarterSecond = shortRunsUnderQuarterSecond;
            copy.totalDurationSec = totalDurationSec;
            copy.maxDurationSec = maxDurationSec;
            return copy;
        }
    }

    private record FeedWindowSummary(
            String fileName,
            int index,
            double startTimeSec,
            double durationSec,
            String mode,
            double startMatchTimeSec,
            String reason,
            double totalSec,
            double positiveSec,
            double missingSec,
            double missingAfterFirstPositiveSec,
            double maxGapSec,
            double maxGapAfterFirstPositiveSec,
            double startLatencySec) {
        private boolean clean() {
            return startLatencySec <= EARLY_WINDOW_ALLOWANCE_SEC && missingAfterFirstPositiveSec <= 1e-6;
        }

        private String describe() {
            return String.format(Locale.US,
                    "%s window#%d t=%.3f dur=%.3fs mode=%s match=%.2f reason=%s startLatency=%.3fs missing=%.3fs missingAfterSpinup=%.3fs maxGap=%.3fs maxGapAfterSpinup=%.3fs clean=%s",
                    fileName,
                    index,
                    startTimeSec,
                    durationSec,
                    mode,
                    startMatchTimeSec,
                    reason,
                    startLatencySec,
                    missingSec,
                    missingAfterFirstPositiveSec,
                    maxGapSec,
                    maxGapAfterFirstPositiveSec,
                    clean());
        }
    }

    private record TransferSegmentSummary(
            String fileName,
            int index,
            double startTimeSec,
            double durationSec,
            String mode,
            double startMatchTimeSec,
            double endMatchTimeSec,
            String reason,
            boolean gateOpenAtStart,
            double startBatteryVoltage,
            double positionDeltaRot,
            WeightedStats currentStats,
            WeightedStats absVelocityStats,
            WeightedStats appliedVoltsStats,
            WeightedStats batteryStats,
            double maxAbsCurrentAmps,
            double minAbsVelocityRpm,
            double maxAbsVelocityRpm,
            int maxJamCurrentCycles,
            boolean jamDetected) {
        private String describe() {
            return String.format(Locale.US,
                    "%s seg#%d t=%.3f dur=%.3fs mode=%s matchStart=%.2f->%.2f reason=%s gateOpenStart=%s battStart=%.2fV posDeltaRot=%s current[%s] absVel[%s] volts[%s] maxAbsCurrent=%.2f minAbsVel=%.2f maxAbsVel=%.2f maxJamCycles=%d jamDetected=%s",
                    fileName,
                    index,
                    startTimeSec,
                    durationSec,
                    mode,
                    startMatchTimeSec,
                    endMatchTimeSec,
                    reason,
                    gateOpenAtStart,
                    startBatteryVoltage,
                    Double.isFinite(positionDeltaRot) ? String.format(Locale.US, "%.3f", positionDeltaRot) : "NaN",
                    currentStats.describe(),
                    absVelocityStats.describe(),
                    appliedVoltsStats.describe(),
                    maxAbsCurrentAmps,
                    Double.isFinite(minAbsVelocityRpm) ? minAbsVelocityRpm : Double.NaN,
                    maxAbsVelocityRpm,
                    maxJamCurrentCycles,
                    jamDetected);
        }
    }

    private record TransferMatchSummary(
            String wpilogPath,
            String fileName,
            boolean truncated,
            List<String> parseWarnings,
            List<Double> nonzeroCommandedPercents,
            double enabledSec,
            double autoSec,
            double teleopSec,
            double expectedFeedSec,
            double alignedPositiveSec,
            double missingExpectedCommandSec,
            double unexplainedPositiveSec,
            double disabledPositiveSec,
            double totalPositiveSec,
            double totalReverseSec,
            double highCurrentWhilePositiveSec,
            double lowVelocityWhilePositiveSec,
            double veryLowVelocityWhilePositiveSec,
            double jamDetectedSec,
            int jamDetectedCount,
            WeightedStats positiveCurrentStats,
            WeightedStats positiveVelocityStats,
            WeightedStats positiveAppliedVoltsStats,
            WeightedStats positiveBatteryStats,
            WeightedStats positiveTempStats,
            List<FeedWindowSummary> feedWindows,
            List<TransferSegmentSummary> positiveSegments,
            Map<String, CommandStats> commandStats,
            List<Double> jamDetectedTimestampsSec) {}

    private static final class AggregateSummary {
        private double totalEnabledSec = 0.0;
        private double totalAutoSec = 0.0;
        private double totalTeleopSec = 0.0;
        private double expectedFeedSec = 0.0;
        private double alignedPositiveSec = 0.0;
        private double missingExpectedCommandSec = 0.0;
        private double unexplainedPositiveSec = 0.0;
        private double disabledPositiveSec = 0.0;
        private int totalPositiveSegments = 0;
        private double totalPositiveSec = 0.0;
        private double totalReverseSec = 0.0;
        private double highCurrentWhilePositiveSec = 0.0;
        private double lowVelocityWhilePositiveSec = 0.0;
        private double veryLowVelocityWhilePositiveSec = 0.0;
        private double jamDetectedSec = 0.0;
        private int jamDetectedCount = 0;
        private int feedWindowCount = 0;
        private int cleanFeedWindows = 0;
        private int feedWindowsWithMissingAfterSpinup = 0;
        private double worstFeedWindowMissingAfterSpinupSec = 0.0;
        private double maxFeedWindowStartLatencySec = 0.0;
        private final WeightedStats positiveCurrentStats = new WeightedStats();
        private final WeightedStats positiveVelocityStats = new WeightedStats();
        private final WeightedStats positiveAppliedVoltsStats = new WeightedStats();
        private final WeightedStats positiveBatteryStats = new WeightedStats();
        private final WeightedStats positiveTempStats = new WeightedStats();
        private final Map<String, CommandStats> commandStats = new LinkedHashMap<>();
        private final List<FeedWindowSummary> allFeedWindows = new ArrayList<>();
        private final List<TransferSegmentSummary> allPositiveSegments = new ArrayList<>();

        private static AggregateSummary from(List<TransferMatchSummary> summaries) {
            AggregateSummary aggregate = new AggregateSummary();
            for (TransferMatchSummary summary : summaries) {
                aggregate.totalEnabledSec += summary.enabledSec;
                aggregate.totalAutoSec += summary.autoSec;
                aggregate.totalTeleopSec += summary.teleopSec;
                aggregate.expectedFeedSec += summary.expectedFeedSec;
                aggregate.alignedPositiveSec += summary.alignedPositiveSec;
                aggregate.missingExpectedCommandSec += summary.missingExpectedCommandSec;
                aggregate.unexplainedPositiveSec += summary.unexplainedPositiveSec;
                aggregate.disabledPositiveSec += summary.disabledPositiveSec;
                aggregate.totalPositiveSegments += summary.positiveSegments.size();
                aggregate.totalPositiveSec += summary.totalPositiveSec;
                aggregate.totalReverseSec += summary.totalReverseSec;
                aggregate.highCurrentWhilePositiveSec += summary.highCurrentWhilePositiveSec;
                aggregate.lowVelocityWhilePositiveSec += summary.lowVelocityWhilePositiveSec;
                aggregate.veryLowVelocityWhilePositiveSec += summary.veryLowVelocityWhilePositiveSec;
                aggregate.jamDetectedSec += summary.jamDetectedSec;
                aggregate.jamDetectedCount += summary.jamDetectedCount;
                aggregate.accumulateStats(aggregate.positiveCurrentStats, summary.positiveCurrentStats);
                aggregate.accumulateStats(aggregate.positiveVelocityStats, summary.positiveVelocityStats);
                aggregate.accumulateStats(aggregate.positiveAppliedVoltsStats, summary.positiveAppliedVoltsStats);
                aggregate.accumulateStats(aggregate.positiveBatteryStats, summary.positiveBatteryStats);
                aggregate.accumulateStats(aggregate.positiveTempStats, summary.positiveTempStats);
                aggregate.feedWindowCount += summary.feedWindows.size();
                for (FeedWindowSummary window : summary.feedWindows) {
                    aggregate.allFeedWindows.add(window);
                    if (window.clean()) {
                        aggregate.cleanFeedWindows++;
                    }
                    if (window.missingAfterFirstPositiveSec() > 1e-6) {
                        aggregate.feedWindowsWithMissingAfterSpinup++;
                    }
                    aggregate.worstFeedWindowMissingAfterSpinupSec = Math.max(
                            aggregate.worstFeedWindowMissingAfterSpinupSec,
                            window.missingAfterFirstPositiveSec());
                    aggregate.maxFeedWindowStartLatencySec = Math.max(
                            aggregate.maxFeedWindowStartLatencySec,
                            window.startLatencySec());
                }
                aggregate.allPositiveSegments.addAll(summary.positiveSegments);
                for (CommandStats stats : summary.commandStats.values()) {
                    CommandStats aggregateStats = aggregate.commandStats.computeIfAbsent(stats.name, CommandStats::new);
                    aggregateStats.transferRequirement |= stats.transferRequirement;
                    aggregateStats.directPositive |= stats.directPositive;
                    aggregateStats.starts += stats.starts;
                    aggregateStats.finishes += stats.finishes;
                    aggregateStats.interrupts += stats.interrupts;
                    aggregateStats.shortRunsUnderQuarterSecond += stats.shortRunsUnderQuarterSecond;
                    aggregateStats.totalDurationSec += stats.totalDurationSec;
                    aggregateStats.maxDurationSec = Math.max(aggregateStats.maxDurationSec, stats.maxDurationSec);
                }
            }
            return aggregate;
        }

        private void accumulateStats(WeightedStats aggregate, WeightedStats incoming) {
            aggregate.totalWeight += incoming.totalWeight;
            aggregate.weightedSum += incoming.weightedSum;
            aggregate.min = Math.min(aggregate.min, incoming.min);
            aggregate.max = Math.max(aggregate.max, incoming.max);
        }

        private List<FeedWindowSummary> worstFeedWindows() {
            List<FeedWindowSummary> sorted = new ArrayList<>(allFeedWindows);
            sorted.sort(Comparator.comparingDouble(FeedWindowSummary::missingAfterFirstPositiveSec)
                    .thenComparingDouble(FeedWindowSummary::startLatencySec)
                    .reversed());
            return sorted.subList(0, Math.min(8, sorted.size()));
        }

        private List<TransferSegmentSummary> worstPositiveSegments() {
            List<TransferSegmentSummary> sorted = new ArrayList<>(allPositiveSegments);
            sorted.sort(Comparator.<TransferSegmentSummary>comparingDouble(segment -> segment.maxAbsCurrentAmps)
                    .thenComparingDouble(segment -> -segment.absVelocityStats.mean())
                    .reversed());
            return sorted.subList(0, Math.min(8, sorted.size()));
        }

        private String verdict() {
            if (missingExpectedCommandSec > 1e-6) {
                return String.format(Locale.US,
                        "NOT clean: expected transfer command was missing for %.3fs total, with worst post-spinup missing window %.3fs.",
                        missingExpectedCommandSec,
                        worstFeedWindowMissingAfterSpinupSec);
            }
            if (unexplainedPositiveSec > 1e-6) {
                return String.format(Locale.US,
                        "Mostly clean, but transfer ran unexpectedly for %.3fs outside gate-open/direct-command windows.",
                        unexplainedPositiveSec);
            }
            if (jamDetectedCount > 0) {
                return String.format(Locale.US,
                        "Command tracking is clean, but jam detection fired %d times and needs inspection.",
                        jamDetectedCount);
            }
            return String.format(Locale.US,
                    "Commanding looks clean across all analyzed matches: %.0f/%.0f feed windows had no post-spinup gaps, no unexplained positive command time, and no jam detections.",
                    (double) cleanFeedWindows,
                    (double) feedWindowCount);
        }
    }
}
