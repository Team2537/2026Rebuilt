package frc.robot.diagnostics;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import frc.robot.coordination.shooting.ShootCoordinatorConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.transfer.TransferConstants;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.junit.jupiter.api.Test;

class MdBetShotGateAuditTest {
    private static final Pattern QUAL_MATCH_PATTERN = Pattern.compile("_q(\\d+)\\.wpilog$");
    private static final double EPS = 1e-6;
    private static final int MAX_EXAMPLES_PER_CATEGORY = 12;

    @Test
    void auditShotGateAcrossMdBetMatchesAfterQ7() throws Exception {
        Path logDir = Path.of("logs/mdbet").toAbsolutePath();
        assertTrue(Files.isDirectory(logDir), "Missing log directory: " + logDir);

        List<Path> logs;
        try (var stream = Files.list(logDir)) {
            logs = stream
                    .filter(path -> Files.isRegularFile(path) && includeAfterMatch7(path))
                    .sorted(Comparator.comparing(Path::toString))
                    .toList();
        }
        assertTrue(!logs.isEmpty(), "No MDBet qualification logs after q7 found in " + logDir);

        List<MatchAudit> audits = new ArrayList<>();
        for (Path log : logs) {
            audits.add(analyzeLog(log));
        }

        String report = formatReport(audits);
        Path out = Path.of("build/reports/shot-gate/mdbet-shot-gate-audit.txt").toAbsolutePath();
        Files.createDirectories(out.getParent());
        Files.writeString(out, report);

        System.out.println("Shot gate audit written: " + out);
        System.out.println(report);
    }

    private static boolean includeAfterMatch7(Path path) {
        Matcher matcher = QUAL_MATCH_PATTERN.matcher(path.getFileName().toString());
        return matcher.find() && Integer.parseInt(matcher.group(1)) > 7;
    }

    private static MatchAudit analyzeLog(Path logPath) throws IOException {
        EntryIds entryIds = scanEntries(logPath);
        if (entryIds.relevantEntryIds().isEmpty()) {
            throw new IllegalStateException("No relevant shot-gate entries found in " + logPath);
        }

        RunningState running = new RunningState(logPath, entryIds);
        DataLogReader reader = new DataLogReader(logPath.toString());
        if (!reader.isValid()) {
            throw new IllegalStateException("Invalid WPILOG file: " + logPath);
        }

        Iterator<DataLogRecord> iterator = reader.iterator();
        long currentTimestampUs = Long.MIN_VALUE;
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
            if (!entryIds.relevantEntryIds().contains(record.getEntry())) {
                continue;
            }

            long timestampUs = record.getTimestamp();
            if (currentTimestampUs != Long.MIN_VALUE && timestampUs != currentTimestampUs) {
                running.finalizeTimestamp(currentTimestampUs);
            }
            currentTimestampUs = timestampUs;
            running.apply(record);
        }

        if (currentTimestampUs != Long.MIN_VALUE) {
            running.finalizeTimestamp(currentTimestampUs);
        }
        running.finish();
        return running.toAudit();
    }

    private static EntryIds scanEntries(Path logPath) throws IOException {
        DataLogReader reader = new DataLogReader(logPath.toString());
        if (!reader.isValid()) {
            throw new IllegalStateException("Invalid WPILOG file: " + logPath);
        }

        Map<String, Integer> entries = new HashMap<>();
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
            String name = start.name;
            putIfMatches(entries, "RobotState/Enabled", name, start.entry);
            putIfMatches(entries, "RobotState/Mode", name, start.entry);
            putIfMatches(entries, "RobotState/MatchTime", name, start.entry);
            putIfMatches(entries, "Shooting/DistanceValid", name, start.entry);
            putIfMatches(entries, "Shooting/FeedGateMode", name, start.entry);
            putIfMatches(entries, "Shooting/Intent", name, start.entry);
            putIfMatches(entries, "Shooting/DistanceMeters", name, start.entry);
            putIfMatches(entries, "Shooting/AimErrorDeg", name, start.entry);
            putIfMatches(entries, "Shooting/AimToleranceRad", name, start.entry);
            putIfMatches(entries, "Shooting/ShooterRpmTolerance", name, start.entry);
            putIfMatches(entries, "Shooting/ShooterAimOnly", name, start.entry);
            putIfMatches(entries, "Shooting/ShooterAtSetpoint", name, start.entry);
            putIfMatches(entries, "Shooting/AimReady", name, start.entry);
            putIfMatches(entries, "Shooting/ManualFeedOverride", name, start.entry);
            putIfMatches(entries, "Shooting/AutomaticFeedEnabled", name, start.entry);
            putIfMatches(entries, "Shooting/GateOpen", name, start.entry);
            putIfMatches(entries, "Shooting/BlockReason", name, start.entry);
            putIfMatches(entries, "Shooting/ReadyStableCycles", name, start.entry);
            putIfMatches(entries, "Shooting/ReadyDropStableCycles", name, start.entry);
            putIfMatches(entries, "Shooting/State", name, start.entry);
            putIfMatches(entries, "Shooting/GateReadyDebounceCycles", name, start.entry);
            putIfMatches(entries, "Shooting/MovingGateDropDebounceCycles", name, start.entry);
            putIfMatches(entries, "Shooter/Readiness/Mode", name, start.entry);
            putIfMatches(entries, "Shooter/Readiness/LeftVelocityErrorRpm", name, start.entry);
            putIfMatches(entries, "Shooter/Readiness/RightVelocityErrorRpm", name, start.entry);
            putIfMatches(entries, "Shooter/Readiness/HoodAngleErrorDeg", name, start.entry);
            putIfMatches(entries, "Shooter/KickerTorqueAmps", name, start.entry);
            putIfMatches(entries, "Transfer/CommandedPercent", name, start.entry);
            putIfMatches(entries, "Commands/lastEvent", name, start.entry);
        }
        return new EntryIds(entries);
    }

    private static void putIfMatches(Map<String, Integer> entries, String suffix, String fullName, int entry) {
        if (fullName != null && (fullName.equals(suffix) || fullName.endsWith("/" + suffix))) {
            entries.putIfAbsent(suffix, entry);
        }
    }

    private static String formatReport(List<MatchAudit> audits) {
        StringBuilder out = new StringBuilder();
        out.append("MDBet shot gate audit (qualification matches after q7)\n");
        out.append("Analyzed logs: ");
        for (int i = 0; i < audits.size(); i++) {
            if (i > 0) {
                out.append(", ");
            }
            out.append(audits.get(i).matchLabel());
        }
        out.append("\n\n");

        Aggregate aggregate = new Aggregate();
        for (MatchAudit audit : audits) {
            aggregate.add(audit);
            formatMatch(out, audit);
            out.append("\n");
        }
        formatAggregate(out, aggregate);
        return out.toString();
    }

    private static void formatMatch(StringBuilder out, MatchAudit audit) {
        out.append("=== ").append(audit.matchLabel()).append(" ===\n");
        out.append("log=").append(audit.logPath().toAbsolutePath()).append('\n');
        out.append(String.format(Locale.US,
                "enabled_sec=%.3f auto_feed_enabled_sec=%.3f manual_override_sec=%.3f gate_open_sec=%.3f active_non_idle_sec=%.3f%n",
                audit.enabledSec(),
                audit.autoFeedEnabledSec(),
                audit.manualOverrideSec(),
                audit.gateOpenSec(),
                audit.activeNonIdleSec()));
        out.append(String.format(Locale.US,
                "samples.active=%d gate_open_rises=%d gate_open_falls=%d shoot_sessions=%d sessions_with_multiple_gate_bursts=%d gate_segments=%d%n",
                audit.activeSamples(),
                audit.gateOpenRiseCount(),
                audit.gateOpenFallCount(),
                audit.sessions().size(),
                audit.sessionsWithMultipleGateBursts(),
                audit.gateSegments().size()));
        out.append("feed_gate_modes_seen=").append(audit.feedGateModesSeen()).append('\n');
        out.append("readiness_modes_seen=").append(audit.readinessModesSeen()).append('\n');
        out.append("intent_seen=").append(audit.intentsSeen()).append('\n');
        out.append(String.format(Locale.US,
                "gate_replay_mismatches=%d actuator_mismatches=%d unexpected_gate_open_samples=%d allowed_moving_drop_open_samples=%d gate_open_while_disabled_samples=%d kicker_edge_lag_samples=%d transfer_edge_lag_samples=%d%n",
                audit.replayMismatches().size(),
                audit.actuatorMismatches().size(),
                audit.unexpectedGateOpenSamples(),
                audit.allowedMovingDropOpenSamples(),
                audit.gateOpenWhileDisabledSamples(),
                audit.kickerEdgeLagSamples(),
                audit.transferEdgeLagSamples()));
        out.append(String.format(Locale.US,
                "command_ended_transitions=%d session_end_interrupt_events=%d interrupt_events_during_active_shoot=%d%n",
                audit.commandEndedTransitions(),
                audit.sessionEndInterruptEvents().size(),
                audit.interruptEventsDuringActiveShoot().size()));
        out.append(String.format(Locale.US,
                "gate_segment_duration_sec[min/median/p95/max]=%.3f / %.3f / %.3f / %.3f%n",
                percentile(audit.gateSegmentDurationsSec(), 0.0),
                percentile(audit.gateSegmentDurationsSec(), 0.5),
                percentile(audit.gateSegmentDurationsSec(), 0.95),
                percentile(audit.gateSegmentDurationsSec(), 1.0)));
        out.append("block_reason_transition_counts=\n");
        appendMap(out, audit.blockReasonTransitionCounts(), 2);
        out.append("state_transition_counts=\n");
        appendMap(out, audit.stateTransitionCounts(), 2);
        out.append("gate_close_reason_counts=\n");
        appendMap(out, audit.gateCloseReasonCounts(), 2);

        out.append("key_gate_segments=\n");
        if (audit.gateSegments().isEmpty()) {
            out.append("  (none)\n");
        } else {
            for (GateSegment segment : audit.gateSegments()) {
                out.append(String.format(Locale.US,
                        "  start_t=%.3f match_time=%.1f dur=%.3f readiness=%s intent=%s dist=%.2f aim_err_max=%.2f left_rpm_err_max=%.1f right_rpm_err_max=%.1f hood_err_max=%.2f open_while_not_ready=%d unexpected_open_samples=%d close_reason=%s end_state=%s end_event=%s%n",
                        segment.startSec(),
                        segment.startMatchTimeSec(),
                        segment.durationSec(),
                        segment.readinessMode(),
                        segment.intent(),
                        segment.startDistanceMeters(),
                        segment.maxAbsAimErrorDeg(),
                        segment.maxAbsLeftVelocityErrorRpm(),
                        segment.maxAbsRightVelocityErrorRpm(),
                        segment.maxAbsHoodErrorDeg(),
                        segment.openWhileNotReadySamples(),
                        segment.unexpectedOpenSamples(),
                        blankAsNone(segment.closeReason()),
                        blankAsNone(segment.endState()),
                        blankAsNone(segment.endEvent())));
            }
        }

        out.append("shoot_sessions=\n");
        if (audit.sessions().isEmpty()) {
            out.append("  (none)\n");
        } else {
            for (ShootSession session : audit.sessions()) {
                out.append(String.format(Locale.US,
                        "  start_t=%.3f end_t=%.3f dur=%.3f start_match=%.1f gate_bursts=%d gate_open_sec=%.3f last_state=%s last_block=%s end_event=%s%n",
                        session.startSec(),
                        session.endSec(),
                        session.durationSec(),
                        session.startMatchTimeSec(),
                        session.gateBurstCount(),
                        session.gateOpenSec(),
                        blankAsNone(session.lastState()),
                        blankAsNone(session.lastBlockReason()),
                        blankAsNone(session.endEvent())));
            }
        }

        appendExamples(out, "replay_mismatch_examples", audit.replayMismatches());
        appendExamples(out, "actuator_mismatch_examples", audit.actuatorMismatches());
        appendExamples(out, "session_end_interrupt_events", audit.sessionEndInterruptEvents());
        appendExamples(out, "interrupt_events_during_active_shoot", audit.interruptEventsDuringActiveShoot());
    }

    private static void formatAggregate(StringBuilder out, Aggregate aggregate) {
        out.append("=== aggregate ===\n");
        out.append(String.format(Locale.US,
                "matches=%d enabled_sec=%.3f auto_feed_enabled_sec=%.3f manual_override_sec=%.3f gate_open_sec=%.3f active_non_idle_sec=%.3f%n",
                aggregate.matchCount,
                aggregate.enabledSec,
                aggregate.autoFeedEnabledSec,
                aggregate.manualOverrideSec,
                aggregate.gateOpenSec,
                aggregate.activeNonIdleSec));
        out.append(String.format(Locale.US,
                "active_samples=%d gate_open_rises=%d gate_open_falls=%d shoot_sessions=%d sessions_with_multiple_gate_bursts=%d gate_segments=%d%n",
                aggregate.activeSamples,
                aggregate.gateOpenRiseCount,
                aggregate.gateOpenFallCount,
                aggregate.sessionCount,
                aggregate.sessionsWithMultipleGateBursts,
                aggregate.gateSegmentDurationsSec.size()));
        out.append(String.format(Locale.US,
                "gate_replay_mismatches=%d actuator_mismatches=%d unexpected_gate_open_samples=%d allowed_moving_drop_open_samples=%d gate_open_while_disabled_samples=%d kicker_edge_lag_samples=%d transfer_edge_lag_samples=%d%n",
                aggregate.replayMismatchCount,
                aggregate.actuatorMismatchCount,
                aggregate.unexpectedGateOpenSamples,
                aggregate.allowedMovingDropOpenSamples,
                aggregate.gateOpenWhileDisabledSamples,
                aggregate.kickerEdgeLagSamples,
                aggregate.transferEdgeLagSamples));
        out.append(String.format(Locale.US,
                "command_ended_transitions=%d session_end_interrupt_events=%d interrupt_events_during_active_shoot=%d%n",
                aggregate.commandEndedTransitions,
                aggregate.sessionEndInterruptEvents.size(),
                aggregate.interruptEventsDuringActiveShoot.size()));
        out.append(String.format(Locale.US,
                "gate_segment_duration_sec[min/median/p95/max]=%.3f / %.3f / %.3f / %.3f%n",
                percentile(aggregate.gateSegmentDurationsSec, 0.0),
                percentile(aggregate.gateSegmentDurationsSec, 0.5),
                percentile(aggregate.gateSegmentDurationsSec, 0.95),
                percentile(aggregate.gateSegmentDurationsSec, 1.0)));
        out.append("feed_gate_modes_seen=").append(aggregate.feedGateModesSeen).append('\n');
        out.append("readiness_modes_seen=").append(aggregate.readinessModesSeen).append('\n');
        out.append("intent_seen=").append(aggregate.intentsSeen).append('\n');
        out.append("block_reason_transition_counts=\n");
        appendMap(out, aggregate.blockReasonTransitionCounts, 2);
        out.append("state_transition_counts=\n");
        appendMap(out, aggregate.stateTransitionCounts, 2);
        out.append("gate_close_reason_counts=\n");
        appendMap(out, aggregate.gateCloseReasonCounts, 2);
        appendExamples(out, "session_end_interrupt_events", aggregate.sessionEndInterruptEvents);
        appendExamples(out, "interrupt_events_during_active_shoot", aggregate.interruptEventsDuringActiveShoot);
    }

    private static void appendMap(StringBuilder out, Map<String, Integer> counts, int indent) {
        if (counts.isEmpty()) {
            out.append(" ".repeat(indent)).append("(none)\n");
            return;
        }
        List<Map.Entry<String, Integer>> entries = new ArrayList<>(counts.entrySet());
        entries.sort((a, b) -> {
            int byValue = Integer.compare(b.getValue(), a.getValue());
            return byValue != 0 ? byValue : a.getKey().compareTo(b.getKey());
        });
        for (Map.Entry<String, Integer> entry : entries) {
            out.append(" ".repeat(indent))
                    .append(blankAsNone(entry.getKey()))
                    .append(": ")
                    .append(entry.getValue())
                    .append('\n');
        }
    }

    private static void appendExamples(StringBuilder out, String label, List<String> examples) {
        out.append(label).append("=\n");
        if (examples.isEmpty()) {
            out.append("  (none)\n");
            return;
        }
        for (String example : examples) {
            out.append("  ").append(example).append('\n');
        }
    }

    private static String blankAsNone(String value) {
        return value == null || value.isBlank() ? "<none>" : value;
    }

    private static double percentile(List<Double> values, double q) {
        if (values.isEmpty()) {
            return 0.0;
        }
        List<Double> sorted = new ArrayList<>(values);
        Collections.sort(sorted);
        if (q <= 0.0) {
            return sorted.get(0);
        }
        if (q >= 1.0) {
            return sorted.get(sorted.size() - 1);
        }
        double index = q * (sorted.size() - 1);
        int lower = (int) Math.floor(index);
        int upper = (int) Math.ceil(index);
        if (lower == upper) {
            return sorted.get(lower);
        }
        double fraction = index - lower;
        return sorted.get(lower) * (1.0 - fraction) + sorted.get(upper) * fraction;
    }

    private static final class EntryIds {
        private final Map<String, Integer> bySuffix;
        private final List<Integer> relevantEntryIds;

        private EntryIds(Map<String, Integer> bySuffix) {
            this.bySuffix = Map.copyOf(bySuffix);
            this.relevantEntryIds = new ArrayList<>(bySuffix.values());
        }

        int get(String suffix) {
            return bySuffix.getOrDefault(suffix, -1);
        }

        List<Integer> relevantEntryIds() {
            return relevantEntryIds;
        }
    }

    private static final class CurrentSample {
        boolean enabled = false;
        String robotMode = "";
        double matchTimeSec = Double.NaN;
        boolean distanceValid = false;
        String feedGateMode = ShootCoordinatorConstants.DEFAULT_FEED_GATE_MODE.name();
        String intent = "";
        double distanceMeters = Double.NaN;
        double aimErrorDeg = Double.NaN;
        double aimToleranceRad = Double.NaN;
        double shooterRpmTolerance = Double.NaN;
        boolean shooterAimOnly = false;
        boolean shooterAtSetpoint = false;
        boolean aimReady = false;
        boolean manualFeedOverride = false;
        boolean automaticFeedEnabled = false;
        boolean gateOpen = false;
        String blockReason = "";
        int readyStableCycles = 0;
        int readyDropStableCycles = 0;
        String state = "";
        int gateReadyDebounceCycles = ShootCoordinatorConstants.gateReadyDebounceCycles();
        int movingGateDropDebounceCycles = ShootCoordinatorConstants.movingGateDropDebounceCycles();
        String shooterReadinessMode = "";
        double leftVelocityErrorRpm = Double.NaN;
        double rightVelocityErrorRpm = Double.NaN;
        double hoodAngleErrorDeg = Double.NaN;
        double kickerTorqueAmps = 0.0;
        double transferPercent = 0.0;
        String lastEvent = "";
    }

    private record Snapshot(
            long timestampUs,
            double timestampSec,
            boolean enabled,
            String robotMode,
            double matchTimeSec,
            boolean distanceValid,
            String feedGateMode,
            String intent,
            double distanceMeters,
            double aimErrorDeg,
            double aimToleranceRad,
            double shooterRpmTolerance,
            boolean shooterAimOnly,
            boolean shooterAtSetpoint,
            boolean aimReady,
            boolean manualFeedOverride,
            boolean automaticFeedEnabled,
            boolean gateOpen,
            String blockReason,
            int readyStableCycles,
            int readyDropStableCycles,
            String state,
            int gateReadyDebounceCycles,
            int movingGateDropDebounceCycles,
            String shooterReadinessMode,
            double leftVelocityErrorRpm,
            double rightVelocityErrorRpm,
            double hoodAngleErrorDeg,
            double kickerTorqueAmps,
            double transferPercent,
            String lastEvent) {
        boolean nonIdleShootState() {
            return state != null && !state.isBlank() && !"IDLE".equals(state);
        }

        boolean movingShot() {
            return "MOVING_SCORE".equals(shooterReadinessMode);
        }
    }

    private static final class GateReplay {
        private int readyStableCycles = 0;
        private int readyDropStableCycles = 0;
        private boolean gateOpen = false;

        void reset() {
            readyStableCycles = 0;
            readyDropStableCycles = 0;
            gateOpen = false;
        }

        GateExpectation update(
                String feedGateModeName,
                int gateReadyDebounceCycles,
                int movingGateDropDebounceCycles,
                boolean solutionValid,
                boolean movingShot,
                boolean shooterAtSetpoint,
                boolean aimReady,
                boolean manualFeedOverride,
                boolean automaticFeedEnabled) {
            ShootCoordinatorConstants.FeedGateMode feedGateMode = parseGateMode(feedGateModeName);
            if (!solutionValid) {
                reset();
                return new GateExpectation(false, "NoValidShotSolution", readyStableCycles, readyDropStableCycles);
            }
            if (manualFeedOverride) {
                reset();
                gateOpen = true;
                return new GateExpectation(true, "ManualFeedOverride", readyStableCycles, readyDropStableCycles);
            }
            if (!automaticFeedEnabled) {
                reset();
                return new GateExpectation(false, "FeedingDisabledOverride", readyStableCycles, readyDropStableCycles);
            }
            if (feedGateMode == ShootCoordinatorConstants.FeedGateMode.IMMEDIATE) {
                readyStableCycles = 0;
                gateOpen = true;
                return new GateExpectation(true, "", readyStableCycles, readyDropStableCycles);
            }

            boolean ready = switch (feedGateMode) {
                case IMMEDIATE -> true;
                case SHOOTER_AT_SETPOINT -> shooterAtSetpoint;
                case SHOOTER_AND_AIM -> shooterAtSetpoint && aimReady;
            };
            if (ready) {
                readyStableCycles++;
                readyDropStableCycles = 0;
                gateOpen = readyStableCycles >= Math.max(1, gateReadyDebounceCycles);
                return new GateExpectation(gateOpen, gateOpen ? "" : "ReadinessDebounce", readyStableCycles, readyDropStableCycles);
            }

            readyStableCycles = 0;
            if (movingShot && gateOpen) {
                readyDropStableCycles++;
                if (readyDropStableCycles < Math.max(1, movingGateDropDebounceCycles)) {
                    return new GateExpectation(true, "MovingShotReadinessDropDebounce", readyStableCycles, readyDropStableCycles);
                }
            }

            readyDropStableCycles = 0;
            gateOpen = false;
            if (!shooterAtSetpoint && !aimReady) {
                return new GateExpectation(false, "ShooterNotAtSetpoint+AimNotReady", readyStableCycles, readyDropStableCycles);
            }
            if (!shooterAtSetpoint) {
                return new GateExpectation(false, "ShooterNotAtSetpoint", readyStableCycles, readyDropStableCycles);
            }
            return new GateExpectation(false, "AimNotReady", readyStableCycles, readyDropStableCycles);
        }

        private static ShootCoordinatorConstants.FeedGateMode parseGateMode(String raw) {
            if (raw == null || raw.isBlank()) {
                return ShootCoordinatorConstants.DEFAULT_FEED_GATE_MODE;
            }
            try {
                return ShootCoordinatorConstants.FeedGateMode.valueOf(raw);
            } catch (IllegalArgumentException ignored) {
                return ShootCoordinatorConstants.DEFAULT_FEED_GATE_MODE;
            }
        }
    }

    private record GateExpectation(boolean gateOpen, String blockReason, int readyStableCycles, int readyDropStableCycles) {}

    private static final class MutableGateSegment {
        final double startSec;
        final double startMatchTimeSec;
        final String readinessMode;
        final String intent;
        final double startDistanceMeters;
        double endSec;
        String closeReason = "";
        String endState = "";
        String endEvent = "";
        double maxAbsAimErrorDeg = 0.0;
        double maxAbsLeftVelocityErrorRpm = 0.0;
        double maxAbsRightVelocityErrorRpm = 0.0;
        double maxAbsHoodErrorDeg = 0.0;
        int openWhileNotReadySamples = 0;
        int unexpectedOpenSamples = 0;

        MutableGateSegment(Snapshot snapshot) {
            this.startSec = snapshot.timestampSec();
            this.startMatchTimeSec = snapshot.matchTimeSec();
            this.readinessMode = blankAsNone(snapshot.shooterReadinessMode());
            this.intent = blankAsNone(snapshot.intent());
            this.startDistanceMeters = snapshot.distanceMeters();
            update(snapshot, false, false);
        }

        void update(Snapshot snapshot, boolean openWhileNotReady, boolean unexpectedOpen) {
            this.endSec = snapshot.timestampSec();
            if (Double.isFinite(snapshot.aimErrorDeg())) {
                maxAbsAimErrorDeg = Math.max(maxAbsAimErrorDeg, Math.abs(snapshot.aimErrorDeg()));
            }
            if (Double.isFinite(snapshot.leftVelocityErrorRpm())) {
                maxAbsLeftVelocityErrorRpm = Math.max(maxAbsLeftVelocityErrorRpm, Math.abs(snapshot.leftVelocityErrorRpm()));
            }
            if (Double.isFinite(snapshot.rightVelocityErrorRpm())) {
                maxAbsRightVelocityErrorRpm = Math.max(maxAbsRightVelocityErrorRpm, Math.abs(snapshot.rightVelocityErrorRpm()));
            }
            if (Double.isFinite(snapshot.hoodAngleErrorDeg())) {
                maxAbsHoodErrorDeg = Math.max(maxAbsHoodErrorDeg, Math.abs(snapshot.hoodAngleErrorDeg()));
            }
            if (openWhileNotReady) {
                openWhileNotReadySamples++;
            }
            if (unexpectedOpen) {
                unexpectedOpenSamples++;
            }
        }

        GateSegment finish(Snapshot snapshot, String closeReason, String endEvent) {
            this.endSec = snapshot.timestampSec();
            this.closeReason = Objects.requireNonNullElse(closeReason, "");
            this.endState = Objects.requireNonNullElse(snapshot.state(), "");
            this.endEvent = Objects.requireNonNullElse(endEvent, "");
            return new GateSegment(
                    startSec,
                    startMatchTimeSec,
                    Math.max(0.0, endSec - startSec),
                    readinessMode,
                    intent,
                    startDistanceMeters,
                    maxAbsAimErrorDeg,
                    maxAbsLeftVelocityErrorRpm,
                    maxAbsRightVelocityErrorRpm,
                    maxAbsHoodErrorDeg,
                    openWhileNotReadySamples,
                    unexpectedOpenSamples,
                    this.closeReason,
                    this.endState,
                    this.endEvent);
        }
    }

    private record GateSegment(
            double startSec,
            double startMatchTimeSec,
            double durationSec,
            String readinessMode,
            String intent,
            double startDistanceMeters,
            double maxAbsAimErrorDeg,
            double maxAbsLeftVelocityErrorRpm,
            double maxAbsRightVelocityErrorRpm,
            double maxAbsHoodErrorDeg,
            int openWhileNotReadySamples,
            int unexpectedOpenSamples,
            String closeReason,
            String endState,
            String endEvent) {}

    private static final class MutableShootSession {
        final double startSec;
        final double startMatchTimeSec;
        double endSec;
        double gateOpenSec = 0.0;
        int gateBurstCount = 0;
        String lastState = "";
        String lastBlockReason = "";
        String endEvent = "";

        MutableShootSession(Snapshot snapshot) {
            this.startSec = snapshot.timestampSec();
            this.startMatchTimeSec = snapshot.matchTimeSec();
            this.endSec = snapshot.timestampSec();
            this.lastState = snapshot.state();
            this.lastBlockReason = snapshot.blockReason();
        }

        void update(Snapshot snapshot) {
            this.endSec = snapshot.timestampSec();
            this.lastState = snapshot.state();
            this.lastBlockReason = snapshot.blockReason();
        }

        ShootSession finish(Snapshot snapshot, String endEvent) {
            this.endSec = snapshot.timestampSec();
            this.lastState = snapshot.state();
            this.lastBlockReason = snapshot.blockReason();
            this.endEvent = Objects.requireNonNullElse(endEvent, "");
            return new ShootSession(
                    startSec,
                    endSec,
                    Math.max(0.0, endSec - startSec),
                    startMatchTimeSec,
                    gateBurstCount,
                    gateOpenSec,
                    lastState,
                    lastBlockReason,
                    this.endEvent);
        }
    }

    private record ShootSession(
            double startSec,
            double endSec,
            double durationSec,
            double startMatchTimeSec,
            int gateBurstCount,
            double gateOpenSec,
            String lastState,
            String lastBlockReason,
            String endEvent) {}

    private record MatchAudit(
            Path logPath,
            String matchLabel,
            double enabledSec,
            double autoFeedEnabledSec,
            double manualOverrideSec,
            double gateOpenSec,
            double activeNonIdleSec,
            int activeSamples,
            int gateOpenRiseCount,
            int gateOpenFallCount,
            int sessionsWithMultipleGateBursts,
            int unexpectedGateOpenSamples,
            int allowedMovingDropOpenSamples,
            int gateOpenWhileDisabledSamples,
            int commandEndedTransitions,
            int kickerEdgeLagSamples,
            int transferEdgeLagSamples,
            List<String> replayMismatches,
            List<String> actuatorMismatches,
            List<String> sessionEndInterruptEvents,
            List<String> interruptEventsDuringActiveShoot,
            Map<String, Integer> blockReasonTransitionCounts,
            Map<String, Integer> stateTransitionCounts,
            Map<String, Integer> gateCloseReasonCounts,
            List<GateSegment> gateSegments,
            List<ShootSession> sessions,
            List<Double> gateSegmentDurationsSec,
            List<String> feedGateModesSeen,
            List<String> readinessModesSeen,
            List<String> intentsSeen) {}

    private static final class RunningState {
        private final Path logPath;
        private final EntryIds entryIds;
        private final CurrentSample current = new CurrentSample();
        private final GateReplay gateReplay = new GateReplay();
        private final List<String> replayMismatches = new ArrayList<>();
        private final List<String> actuatorMismatches = new ArrayList<>();
        private final List<String> sessionEndInterruptEvents = new ArrayList<>();
        private final List<String> interruptEventsDuringActiveShoot = new ArrayList<>();
        private final Map<String, Integer> blockReasonTransitionCounts = new LinkedHashMap<>();
        private final Map<String, Integer> stateTransitionCounts = new LinkedHashMap<>();
        private final Map<String, Integer> gateCloseReasonCounts = new LinkedHashMap<>();
        private final List<GateSegment> gateSegments = new ArrayList<>();
        private final List<ShootSession> sessions = new ArrayList<>();
        private final List<Double> gateSegmentDurationsSec = new ArrayList<>();
        private final List<String> feedGateModesSeen = new ArrayList<>();
        private final List<String> readinessModesSeen = new ArrayList<>();
        private final List<String> intentsSeen = new ArrayList<>();
        private Snapshot previousSnapshot = null;
        private MutableGateSegment activeGateSegment = null;
        private MutableShootSession activeShootSession = null;
        private String newestCommandEventAtTimestamp = "";
        private String lastRecordedCommandEvent = "";
        private double enabledSec = 0.0;
        private double autoFeedEnabledSec = 0.0;
        private double manualOverrideSec = 0.0;
        private double gateOpenSec = 0.0;
        private double activeNonIdleSec = 0.0;
        private int activeSamples = 0;
        private int gateOpenRiseCount = 0;
        private int gateOpenFallCount = 0;
        private int unexpectedGateOpenSamples = 0;
        private int allowedMovingDropOpenSamples = 0;
        private int gateOpenWhileDisabledSamples = 0;
        private int commandEndedTransitions = 0;
        private int kickerEdgeLagSamples = 0;
        private int transferEdgeLagSamples = 0;

        private RunningState(Path logPath, EntryIds entryIds) {
            this.logPath = logPath;
            this.entryIds = entryIds;
        }

        void apply(DataLogRecord record) {
            int entry = record.getEntry();
            if (entry == entryIds.get("RobotState/Enabled")) {
                current.enabled = readBooleanLenient(record);
            } else if (entry == entryIds.get("RobotState/Mode")) {
                current.robotMode = readStringLenient(record);
            } else if (entry == entryIds.get("RobotState/MatchTime")) {
                current.matchTimeSec = readDoubleLenient(record);
            } else if (entry == entryIds.get("Shooting/DistanceValid")) {
                current.distanceValid = readBooleanLenient(record);
            } else if (entry == entryIds.get("Shooting/FeedGateMode")) {
                current.feedGateMode = readStringLenient(record);
            } else if (entry == entryIds.get("Shooting/Intent")) {
                current.intent = readStringLenient(record);
            } else if (entry == entryIds.get("Shooting/DistanceMeters")) {
                current.distanceMeters = readDoubleLenient(record);
            } else if (entry == entryIds.get("Shooting/AimErrorDeg")) {
                current.aimErrorDeg = readDoubleLenient(record);
            } else if (entry == entryIds.get("Shooting/AimToleranceRad")) {
                current.aimToleranceRad = readDoubleLenient(record);
            } else if (entry == entryIds.get("Shooting/ShooterRpmTolerance")) {
                current.shooterRpmTolerance = readDoubleLenient(record);
            } else if (entry == entryIds.get("Shooting/ShooterAimOnly")) {
                current.shooterAimOnly = readBooleanLenient(record);
            } else if (entry == entryIds.get("Shooting/ShooterAtSetpoint")) {
                current.shooterAtSetpoint = readBooleanLenient(record);
            } else if (entry == entryIds.get("Shooting/AimReady")) {
                current.aimReady = readBooleanLenient(record);
            } else if (entry == entryIds.get("Shooting/ManualFeedOverride")) {
                current.manualFeedOverride = readBooleanLenient(record);
            } else if (entry == entryIds.get("Shooting/AutomaticFeedEnabled")) {
                current.automaticFeedEnabled = readBooleanLenient(record);
            } else if (entry == entryIds.get("Shooting/GateOpen")) {
                current.gateOpen = readBooleanLenient(record);
            } else if (entry == entryIds.get("Shooting/BlockReason")) {
                current.blockReason = readStringLenient(record);
            } else if (entry == entryIds.get("Shooting/ReadyStableCycles")) {
                current.readyStableCycles = (int) readLongLenient(record);
            } else if (entry == entryIds.get("Shooting/ReadyDropStableCycles")) {
                current.readyDropStableCycles = (int) readLongLenient(record);
            } else if (entry == entryIds.get("Shooting/State")) {
                current.state = readStringLenient(record);
            } else if (entry == entryIds.get("Shooting/GateReadyDebounceCycles")) {
                current.gateReadyDebounceCycles = (int) Math.round(readDoubleLenient(record));
            } else if (entry == entryIds.get("Shooting/MovingGateDropDebounceCycles")) {
                current.movingGateDropDebounceCycles = (int) Math.round(readDoubleLenient(record));
            } else if (entry == entryIds.get("Shooter/Readiness/Mode")) {
                current.shooterReadinessMode = readStringLenient(record);
            } else if (entry == entryIds.get("Shooter/Readiness/LeftVelocityErrorRpm")) {
                current.leftVelocityErrorRpm = readDoubleLenient(record);
            } else if (entry == entryIds.get("Shooter/Readiness/RightVelocityErrorRpm")) {
                current.rightVelocityErrorRpm = readDoubleLenient(record);
            } else if (entry == entryIds.get("Shooter/Readiness/HoodAngleErrorDeg")) {
                current.hoodAngleErrorDeg = readDoubleLenient(record);
            } else if (entry == entryIds.get("Shooter/KickerTorqueAmps")) {
                current.kickerTorqueAmps = readDoubleLenient(record);
            } else if (entry == entryIds.get("Transfer/CommandedPercent")) {
                current.transferPercent = readDoubleLenient(record);
            } else if (entry == entryIds.get("Commands/lastEvent")) {
                String nextEvent = readStringLenient(record);
                current.lastEvent = nextEvent;
                if (!nextEvent.equals(lastRecordedCommandEvent)) {
                    newestCommandEventAtTimestamp = nextEvent;
                    lastRecordedCommandEvent = nextEvent;
                }
            }
        }

        void finalizeTimestamp(long timestampUs) {
            Snapshot snapshot = new Snapshot(
                    timestampUs,
                    timestampUs / 1_000_000.0,
                    current.enabled,
                    normalize(current.robotMode),
                    current.matchTimeSec,
                    current.distanceValid,
                    normalize(current.feedGateMode),
                    normalize(current.intent),
                    current.distanceMeters,
                    current.aimErrorDeg,
                    current.aimToleranceRad,
                    current.shooterRpmTolerance,
                    current.shooterAimOnly,
                    current.shooterAtSetpoint,
                    current.aimReady,
                    current.manualFeedOverride,
                    current.automaticFeedEnabled,
                    current.gateOpen,
                    normalize(current.blockReason),
                    current.readyStableCycles,
                    current.readyDropStableCycles,
                    normalize(current.state),
                    Math.max(1, current.gateReadyDebounceCycles),
                    Math.max(1, current.movingGateDropDebounceCycles),
                    normalize(current.shooterReadinessMode),
                    current.leftVelocityErrorRpm,
                    current.rightVelocityErrorRpm,
                    current.hoodAngleErrorDeg,
                    current.kickerTorqueAmps,
                    current.transferPercent,
                    normalize(current.lastEvent));

            if (previousSnapshot != null) {
                double dtSec = Math.max(0.0, snapshot.timestampSec() - previousSnapshot.timestampSec());
                if (previousSnapshot.enabled()) {
                    enabledSec += dtSec;
                }
                if (previousSnapshot.automaticFeedEnabled()) {
                    autoFeedEnabledSec += dtSec;
                }
                if (previousSnapshot.manualFeedOverride()) {
                    manualOverrideSec += dtSec;
                }
                if (previousSnapshot.gateOpen()) {
                    gateOpenSec += dtSec;
                    if (activeShootSession != null) {
                        activeShootSession.gateOpenSec += dtSec;
                    }
                }
                if (previousSnapshot.nonIdleShootState()) {
                    activeNonIdleSec += dtSec;
                }
            }

            if (!feedGateModesSeen.contains(snapshot.feedGateMode())) {
                feedGateModesSeen.add(snapshot.feedGateMode());
            }
            if (!snapshot.shooterReadinessMode().isBlank() && !readinessModesSeen.contains(snapshot.shooterReadinessMode())) {
                readinessModesSeen.add(snapshot.shooterReadinessMode());
            }
            if (!snapshot.intent().isBlank() && !intentsSeen.contains(snapshot.intent())) {
                intentsSeen.add(snapshot.intent());
            }

            if (snapshot.nonIdleShootState()) {
                activeSamples++;
            }
            if (snapshot.gateOpen() && !snapshot.enabled()) {
                gateOpenWhileDisabledSamples++;
            }

            if (previousSnapshot == null || !previousSnapshot.state().equals(snapshot.state())) {
                increment(stateTransitionCounts, snapshot.state());
            }
            if (previousSnapshot == null || !previousSnapshot.blockReason().equals(snapshot.blockReason())) {
                increment(blockReasonTransitionCounts, snapshot.blockReason());
            }

            boolean activeShootState = snapshot.nonIdleShootState();
            if (activeShootState && activeShootSession == null) {
                activeShootSession = new MutableShootSession(snapshot);
            }
            if (activeShootSession != null) {
                activeShootSession.update(snapshot);
            }

            if (snapshot.gateOpen() && (previousSnapshot == null || !previousSnapshot.gateOpen())) {
                gateOpenRiseCount++;
                if (activeShootSession != null) {
                    activeShootSession.gateBurstCount++;
                }
                activeGateSegment = new MutableGateSegment(snapshot);
            }

            boolean openWhileNotReady = snapshot.gateOpen()
                    && !snapshot.manualFeedOverride()
                    && (!snapshot.shooterAtSetpoint() || !snapshot.aimReady());
            boolean allowedMovingDropOpen = openWhileNotReady
                    && snapshot.movingShot()
                    && "MovingShotReadinessDropDebounce".equals(snapshot.blockReason())
                    && snapshot.readyDropStableCycles() > 0;
            boolean unexpectedOpen = false;
            if (snapshot.gateOpen()) {
                if (openWhileNotReady) {
                    if (allowedMovingDropOpen) {
                        allowedMovingDropOpenSamples++;
                    } else {
                        unexpectedGateOpenSamples++;
                        unexpectedOpen = true;
                    }
                }
                if (!snapshot.distanceValid() || (!snapshot.automaticFeedEnabled() && !snapshot.manualFeedOverride())) {
                    unexpectedGateOpenSamples++;
                    unexpectedOpen = true;
                }
            }
            if (activeGateSegment != null && snapshot.gateOpen()) {
                activeGateSegment.update(snapshot, openWhileNotReady, unexpectedOpen);
            }

            if (activeShootState) {
                if (previousSnapshot == null || !previousSnapshot.nonIdleShootState()) {
                    gateReplay.reset();
                }
                GateExpectation expectation = gateReplay.update(
                        snapshot.feedGateMode(),
                        snapshot.gateReadyDebounceCycles(),
                        snapshot.movingGateDropDebounceCycles(),
                        snapshot.distanceValid(),
                        snapshot.movingShot(),
                        snapshot.shooterAtSetpoint(),
                        snapshot.aimReady(),
                        snapshot.manualFeedOverride(),
                        snapshot.automaticFeedEnabled());
                if (expectation.gateOpen() != snapshot.gateOpen()
                        || !normalize(expectation.blockReason()).equals(snapshot.blockReason())
                        || expectation.readyStableCycles() != snapshot.readyStableCycles()
                        || expectation.readyDropStableCycles() != snapshot.readyDropStableCycles()) {
                    addLimited(replayMismatches, String.format(Locale.US,
                            "t=%.3f match=%.1f expected[gate=%s block=%s ready=%d drop=%d] actual[gate=%s block=%s ready=%d drop=%d] state=%s auto=%s manual=%s valid=%s aim=%s shooter=%s readinessMode=%s",
                            snapshot.timestampSec(),
                            snapshot.matchTimeSec(),
                            expectation.gateOpen(),
                            blankAsNone(expectation.blockReason()),
                            expectation.readyStableCycles(),
                            expectation.readyDropStableCycles(),
                            snapshot.gateOpen(),
                            blankAsNone(snapshot.blockReason()),
                            snapshot.readyStableCycles(),
                            snapshot.readyDropStableCycles(),
                            blankAsNone(snapshot.state()),
                            snapshot.automaticFeedEnabled(),
                            snapshot.manualFeedOverride(),
                            snapshot.distanceValid(),
                            snapshot.aimReady(),
                            snapshot.shooterAtSetpoint(),
                            blankAsNone(snapshot.shooterReadinessMode())));
                }
            }

            boolean expectedFeeding = snapshot.gateOpen();
            boolean gateChanged = previousSnapshot != null && previousSnapshot.gateOpen() != snapshot.gateOpen();
            boolean kickerMatches = expectedFeeding
                    ? Math.abs(snapshot.kickerTorqueAmps() - ShooterConstants.DEFAULT_KICKER_TORQUE_AMPS) <= 1e-3
                    : Math.abs(snapshot.kickerTorqueAmps()) <= 1e-3;
            boolean transferMatches = expectedFeeding
                    ? Math.abs(snapshot.transferPercent() - TransferConstants.RUN_TRANSFER_PERCENT) <= 1e-3
                    : Math.abs(snapshot.transferPercent()) <= 1e-3;
            if (gateChanged) {
                if (!kickerMatches) {
                    kickerEdgeLagSamples++;
                }
                if (!transferMatches) {
                    transferEdgeLagSamples++;
                }
            }
            boolean kickerMismatch = !gateChanged && !kickerMatches;
            boolean transferMismatch = !gateChanged && !transferMatches;
            if (kickerMismatch || transferMismatch) {
                addLimited(actuatorMismatches, String.format(Locale.US,
                        "t=%.3f match=%.1f gate=%s kicker=%.3f transfer=%.3f expectedKicker=%.1f expectedTransfer=%.2f state=%s block=%s event=%s",
                        snapshot.timestampSec(),
                        snapshot.matchTimeSec(),
                        snapshot.gateOpen(),
                        snapshot.kickerTorqueAmps(),
                        snapshot.transferPercent(),
                        expectedFeeding ? ShooterConstants.DEFAULT_KICKER_TORQUE_AMPS : 0.0,
                        expectedFeeding ? TransferConstants.RUN_TRANSFER_PERCENT : 0.0,
                        blankAsNone(snapshot.state()),
                        blankAsNone(snapshot.blockReason()),
                        blankAsNone(snapshot.lastEvent())));
            }

            if (previousSnapshot != null && previousSnapshot.gateOpen() && !snapshot.gateOpen()) {
                gateOpenFallCount++;
                increment(gateCloseReasonCounts, snapshot.blockReason());
                if (activeGateSegment != null) {
                    GateSegment segment = activeGateSegment.finish(snapshot, snapshot.blockReason(), nearestCommandEvent(snapshot.timestampUs()));
                    gateSegments.add(segment);
                    gateSegmentDurationsSec.add(segment.durationSec());
                    activeGateSegment = null;
                }
            }

            if (previousSnapshot != null && !"CommandEnded".equals(previousSnapshot.blockReason()) && "CommandEnded".equals(snapshot.blockReason())) {
                commandEndedTransitions++;
            }

            if (newestCommandEventAtTimestamp != null && !newestCommandEventAtTimestamp.isBlank()) {
                if (newestCommandEventAtTimestamp.contains("INTERRUPT") && snapshot.nonIdleShootState()) {
                    addLimited(interruptEventsDuringActiveShoot, String.format(
                            Locale.US,
                            "t=%.3f match=%.1f state=%s block=%s gate=%s event=%s",
                            snapshot.timestampSec(),
                            snapshot.matchTimeSec(),
                            blankAsNone(snapshot.state()),
                            blankAsNone(snapshot.blockReason()),
                            snapshot.gateOpen(),
                            newestCommandEventAtTimestamp));
                }
            }

            if (previousSnapshot != null && previousSnapshot.nonIdleShootState() && !snapshot.nonIdleShootState()) {
                String endEvent = nearestCommandEvent(snapshot.timestampUs());
                if (endEvent.contains("INTERRUPT")) {
                    addLimited(sessionEndInterruptEvents, String.format(
                            Locale.US,
                            "t=%.3f match=%.1f last_state=%s last_block=%s event=%s",
                            snapshot.timestampSec(),
                            snapshot.matchTimeSec(),
                            blankAsNone(previousSnapshot.state()),
                            blankAsNone(previousSnapshot.blockReason()),
                            endEvent));
                }
                if (activeShootSession != null) {
                    sessions.add(activeShootSession.finish(previousSnapshot, endEvent));
                    activeShootSession = null;
                }
                gateReplay.reset();
            }

            newestCommandEventAtTimestamp = "";
            previousSnapshot = snapshot;
        }

        void finish() {
            if (previousSnapshot != null && activeGateSegment != null) {
                GateSegment segment = activeGateSegment.finish(previousSnapshot, previousSnapshot.blockReason(), nearestCommandEvent(previousSnapshot.timestampUs()));
                gateSegments.add(segment);
                gateSegmentDurationsSec.add(segment.durationSec());
                activeGateSegment = null;
            }
            if (previousSnapshot != null && activeShootSession != null) {
                sessions.add(activeShootSession.finish(previousSnapshot, nearestCommandEvent(previousSnapshot.timestampUs())));
                activeShootSession = null;
            }
        }

        MatchAudit toAudit() {
            int sessionsWithMultipleGateBursts = 0;
            for (ShootSession session : sessions) {
                if (session.gateBurstCount() > 1) {
                    sessionsWithMultipleGateBursts++;
                }
            }
            return new MatchAudit(
                    logPath,
                    matchLabel(logPath),
                    enabledSec,
                    autoFeedEnabledSec,
                    manualOverrideSec,
                    gateOpenSec,
                    activeNonIdleSec,
                    activeSamples,
                    gateOpenRiseCount,
                    gateOpenFallCount,
                    sessionsWithMultipleGateBursts,
                    unexpectedGateOpenSamples,
                    allowedMovingDropOpenSamples,
                    gateOpenWhileDisabledSamples,
                    commandEndedTransitions,
                    kickerEdgeLagSamples,
                    transferEdgeLagSamples,
                    List.copyOf(replayMismatches),
                    List.copyOf(actuatorMismatches),
                    List.copyOf(sessionEndInterruptEvents),
                    List.copyOf(interruptEventsDuringActiveShoot),
                    Map.copyOf(blockReasonTransitionCounts),
                    Map.copyOf(stateTransitionCounts),
                    Map.copyOf(gateCloseReasonCounts),
                    List.copyOf(gateSegments),
                    List.copyOf(sessions),
                    List.copyOf(gateSegmentDurationsSec),
                    List.copyOf(feedGateModesSeen),
                    List.copyOf(readinessModesSeen),
                    List.copyOf(intentsSeen));
        }

        private String nearestCommandEvent(long timestampUs) {
            if (newestCommandEventAtTimestamp != null && !newestCommandEventAtTimestamp.isBlank()) {
                return newestCommandEventAtTimestamp;
            }
            return current.lastEvent == null ? "" : current.lastEvent;
        }
    }

    private static final class Aggregate {
        int matchCount = 0;
        double enabledSec = 0.0;
        double autoFeedEnabledSec = 0.0;
        double manualOverrideSec = 0.0;
        double gateOpenSec = 0.0;
        double activeNonIdleSec = 0.0;
        int activeSamples = 0;
        int gateOpenRiseCount = 0;
        int gateOpenFallCount = 0;
        int sessionCount = 0;
        int sessionsWithMultipleGateBursts = 0;
        int replayMismatchCount = 0;
        int actuatorMismatchCount = 0;
        int unexpectedGateOpenSamples = 0;
        int allowedMovingDropOpenSamples = 0;
        int gateOpenWhileDisabledSamples = 0;
        int commandEndedTransitions = 0;
        int kickerEdgeLagSamples = 0;
        int transferEdgeLagSamples = 0;
        final List<String> sessionEndInterruptEvents = new ArrayList<>();
        final List<String> interruptEventsDuringActiveShoot = new ArrayList<>();
        final Map<String, Integer> blockReasonTransitionCounts = new LinkedHashMap<>();
        final Map<String, Integer> stateTransitionCounts = new LinkedHashMap<>();
        final Map<String, Integer> gateCloseReasonCounts = new LinkedHashMap<>();
        final List<Double> gateSegmentDurationsSec = new ArrayList<>();
        final List<String> feedGateModesSeen = new ArrayList<>();
        final List<String> readinessModesSeen = new ArrayList<>();
        final List<String> intentsSeen = new ArrayList<>();

        void add(MatchAudit audit) {
            matchCount++;
            enabledSec += audit.enabledSec();
            autoFeedEnabledSec += audit.autoFeedEnabledSec();
            manualOverrideSec += audit.manualOverrideSec();
            gateOpenSec += audit.gateOpenSec();
            activeNonIdleSec += audit.activeNonIdleSec();
            activeSamples += audit.activeSamples();
            gateOpenRiseCount += audit.gateOpenRiseCount();
            gateOpenFallCount += audit.gateOpenFallCount();
            sessionCount += audit.sessions().size();
            sessionsWithMultipleGateBursts += audit.sessionsWithMultipleGateBursts();
            replayMismatchCount += audit.replayMismatches().size();
            actuatorMismatchCount += audit.actuatorMismatches().size();
            unexpectedGateOpenSamples += audit.unexpectedGateOpenSamples();
            allowedMovingDropOpenSamples += audit.allowedMovingDropOpenSamples();
            gateOpenWhileDisabledSamples += audit.gateOpenWhileDisabledSamples();
            commandEndedTransitions += audit.commandEndedTransitions();
            kickerEdgeLagSamples += audit.kickerEdgeLagSamples();
            transferEdgeLagSamples += audit.transferEdgeLagSamples();
            merge(blockReasonTransitionCounts, audit.blockReasonTransitionCounts());
            merge(stateTransitionCounts, audit.stateTransitionCounts());
            merge(gateCloseReasonCounts, audit.gateCloseReasonCounts());
            gateSegmentDurationsSec.addAll(audit.gateSegmentDurationsSec());
            for (String mode : audit.feedGateModesSeen()) {
                if (!feedGateModesSeen.contains(mode)) {
                    feedGateModesSeen.add(mode);
                }
            }
            for (String mode : audit.readinessModesSeen()) {
                if (!readinessModesSeen.contains(mode)) {
                    readinessModesSeen.add(mode);
                }
            }
            for (String intent : audit.intentsSeen()) {
                if (!intentsSeen.contains(intent)) {
                    intentsSeen.add(intent);
                }
            }
            prefixAndAdd(sessionEndInterruptEvents, audit.matchLabel(), audit.sessionEndInterruptEvents());
            prefixAndAdd(interruptEventsDuringActiveShoot, audit.matchLabel(), audit.interruptEventsDuringActiveShoot());
        }
    }

    private static void prefixAndAdd(List<String> target, String prefix, List<String> values) {
        for (String value : values) {
            addLimited(target, prefix + " | " + value);
        }
    }

    private static void merge(Map<String, Integer> target, Map<String, Integer> source) {
        for (Map.Entry<String, Integer> entry : source.entrySet()) {
            target.merge(entry.getKey(), entry.getValue(), Integer::sum);
        }
    }

    private static void increment(Map<String, Integer> counts, String key) {
        counts.merge(blankAsNone(key), 1, Integer::sum);
    }

    private static void addLimited(List<String> target, String value) {
        if (target.size() < MAX_EXAMPLES_PER_CATEGORY) {
            target.add(value);
        }
    }

    private static String normalize(String value) {
        return value == null ? "" : value;
    }

    private static String matchLabel(Path logPath) {
        String fileName = logPath.getFileName().toString();
        Matcher matcher = QUAL_MATCH_PATTERN.matcher(fileName);
        if (matcher.find()) {
            return "q" + matcher.group(1);
        }
        return fileName;
    }

    private static boolean readBooleanLenient(DataLogRecord record) {
        try {
            return record.getBoolean();
        } catch (Exception ignored) {
        }
        try {
            return readLongLenient(record) != 0L;
        } catch (Exception ignored) {
        }
        try {
            return Boolean.parseBoolean(record.getString());
        } catch (Exception ignored) {
        }
        throw new IllegalStateException("Unable to read boolean at entry " + record.getEntry());
    }

    private static String readStringLenient(DataLogRecord record) {
        try {
            return record.getString();
        } catch (Exception ignored) {
        }
        try {
            return String.valueOf(record.getInteger());
        } catch (Exception ignored) {
        }
        try {
            return String.valueOf(record.getDouble());
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
        try {
            return record.getBoolean() ? 1.0 : 0.0;
        } catch (Exception ignored) {
        }
        throw new IllegalStateException("Unable to read double at entry " + record.getEntry());
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
        try {
            return record.getBoolean() ? 1L : 0L;
        } catch (Exception ignored) {
        }
        throw new IllegalStateException("Unable to read integer at entry " + record.getEntry());
    }
}
