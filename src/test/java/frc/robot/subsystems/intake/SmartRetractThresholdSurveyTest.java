package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assumptions.assumeTrue;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.Test;

class SmartRetractThresholdSurveyTest {
    @Test
    void surveySmartRetractThresholdsFromDirectory() throws IOException {
        String directoryPath = System.getProperty("smartRetractSurvey.dir");
        if (directoryPath == null || directoryPath.isBlank()) {
            directoryPath = System.getenv("SMART_RETRACT_SURVEY_DIR");
        }
        assumeTrue(
                directoryPath != null && !directoryPath.isBlank(),
                "Missing survey dir. Set -DsmartRetractSurvey.dir=<dir> or SMART_RETRACT_SURVEY_DIR=<dir>.");

        Path directory = Path.of(directoryPath).toAbsolutePath();
        assumeTrue(Files.isDirectory(directory), "Survey directory does not exist: " + directory);

        List<Path> logs = Files.list(directory)
                .filter(path -> path.getFileName().toString().endsWith(".wpilog"))
                .sorted()
                .toList();
        assumeTrue(!logs.isEmpty(), "No .wpilog files found in: " + directory);

        List<Double> aggregateBackoffFiltered = new ArrayList<>();
        List<Double> aggregateBackoffRaw = new ArrayList<>();
        List<Double> aggregateBackoffBaseline = new ArrayList<>();
        List<Double> aggregateActiveFiltered = new ArrayList<>();
        List<Double> aggregateActiveRaw = new ArrayList<>();
        List<Double> aggregateMinTargets = new ArrayList<>();

        System.out.println("Smart retract threshold survey");
        System.out.println("dir=" + directory);
        for (Path log : logs) {
            SurveySummary summary = analyze(log);
            if (!summary.sawNibbleSession) {
                continue;
            }
            aggregateBackoffFiltered.addAll(summary.backoffFilteredCurrents);
            aggregateBackoffRaw.addAll(summary.backoffRawCurrents);
            aggregateBackoffBaseline.addAll(summary.backoffBaselineCurrents);
            aggregateActiveFiltered.addAll(summary.activeFilteredCurrents);
            aggregateActiveRaw.addAll(summary.activeRawCurrents);
            if (Double.isFinite(summary.minTargetRot)) {
                aggregateMinTargets.add(summary.minTargetRot);
            }
            System.out.println(formatSummary(log, summary));
        }

        System.out.println("Aggregate");
        System.out.printf(Locale.US, "  activeFiltered count=%d %s%n", aggregateActiveFiltered.size(), describe(aggregateActiveFiltered));
        System.out.printf(Locale.US, "  activeRaw      count=%d %s%n", aggregateActiveRaw.size(), describe(aggregateActiveRaw));
        System.out.printf(Locale.US, "  backoffFiltered count=%d %s%n", aggregateBackoffFiltered.size(), describe(aggregateBackoffFiltered));
        System.out.printf(Locale.US, "  backoffRaw      count=%d %s%n", aggregateBackoffRaw.size(), describe(aggregateBackoffRaw));
        System.out.printf(Locale.US, "  backoffBaseline count=%d %s%n", aggregateBackoffBaseline.size(), describe(aggregateBackoffBaseline));
        System.out.printf(Locale.US, "  backoffDelta    count=%d %s%n", aggregateBackoffFiltered.size(), describeDelta(aggregateBackoffFiltered, aggregateBackoffBaseline));
        System.out.printf(Locale.US, "  minTargetRot    count=%d %s%n", aggregateMinTargets.size(), describe(aggregateMinTargets));
        printThresholdCoverage("filtered", aggregateActiveFiltered, aggregateBackoffFiltered, List.of(6.0, 8.0, 10.0, 12.0, 14.0, 16.0));
        printThresholdCoverage("raw", aggregateActiveRaw, aggregateBackoffRaw, List.of(10.0, 12.0, 14.0, 16.0, 18.0, 20.0, 24.0));
    }

    private static SurveySummary analyze(Path wpilog) throws IOException {
        DataLogReader reader = new DataLogReader(wpilog.toString());
        if (!reader.isValid()) {
            throw new IllegalStateException("Invalid WPILOG file: " + wpilog);
        }

        EntryIds entryIds = new EntryIds();
        Iterator<DataLogRecord> startIterator = reader.iterator();
        while (true) {
            DataLogRecord record;
            try {
                if (!startIterator.hasNext()) {
                    break;
                }
                record = startIterator.next();
            } catch (IllegalArgumentException ex) {
                break;
            }
            if (!record.isStart()) {
                continue;
            }
            var start = record.getStartData();
            entryIds.capture(start.entry, start.name);
        }

        SurveyState state = new SurveyState();
        Iterator<DataLogRecord> dataIterator = reader.iterator();
        while (true) {
            DataLogRecord record;
            try {
                if (!dataIterator.hasNext()) {
                    break;
                }
                record = dataIterator.next();
            } catch (IllegalArgumentException ex) {
                state.truncated = true;
                break;
            }
            if (record.isStart() || record.isControl()) {
                continue;
            }

            int entry = record.getEntry();
            if (entry == entryIds.sessionModeEntry) {
                state.sessionMode = record.getString();
                if ("NIBBLE".equals(state.sessionMode)) {
                    state.sawNibbleSession = true;
                }
                continue;
            }
            if (entry == entryIds.sessionActiveEntry) {
                state.sessionActive = record.getBoolean();
                continue;
            }
            if (entry == entryIds.feedLatchedEntry) {
                state.feedLatched = record.getBoolean();
                continue;
            }
            if (entry == entryIds.commandedTargetEntry) {
                state.commandedTargetRot = record.getDouble();
                state.minTargetRot = Math.min(state.minTargetRot, state.commandedTargetRot);
                continue;
            }
            if (entry == entryIds.signalCurrentRawEntry) {
                state.rawCurrentAmps = record.getDouble();
                if (state.inActiveNibbleWindow()) {
                    state.activeRawCurrents.add(state.rawCurrentAmps);
                }
                continue;
            }
            if (entry == entryIds.signalCurrentFilteredEntry) {
                state.filteredCurrentAmps = record.getDouble();
                if (state.inActiveNibbleWindow()) {
                    state.activeFilteredCurrents.add(state.filteredCurrentAmps);
                }
                continue;
            }
            if (entry == entryIds.signalBaselineEntry) {
                state.baselineCurrentAmps = record.getDouble();
                continue;
            }
            if (entry == entryIds.nibbleBackoffActiveEntry) {
                boolean newValue = record.getBoolean();
                if (newValue && !state.nibbleBackoffActive && state.inActiveNibbleWindow()) {
                    state.backoffFilteredCurrents.add(state.filteredCurrentAmps);
                    state.backoffRawCurrents.add(state.rawCurrentAmps);
                    state.backoffBaselineCurrents.add(state.baselineCurrentAmps);
                    state.backoffTargets.add(state.commandedTargetRot);
                }
                state.nibbleBackoffActive = newValue;
            }
        }

        return new SurveySummary(
                state.sawNibbleSession,
                state.truncated,
                state.minTargetRot,
                state.activeFilteredCurrents,
                state.activeRawCurrents,
                state.backoffFilteredCurrents,
                state.backoffRawCurrents,
                state.backoffBaselineCurrents,
                state.backoffTargets);
    }

    private static String formatSummary(Path wpilog, SurveySummary summary) {
        return String.format(
                Locale.US,
                "  %s%n"
                        + "    truncated=%s%n"
                        + "    minTargetRot=%.3f%n"
                        + "    activeFiltered %s%n"
                        + "    backoffFiltered %s%n"
                        + "    backoffRaw      %s%n"
                        + "    backoffBaseline %s%n"
                        + "    backoffDelta    %s%n"
                        + "    backoffTargets  %s",
                wpilog.getFileName(),
                summary.truncated,
                summary.minTargetRot,
                describe(summary.activeFilteredCurrents),
                describe(summary.backoffFilteredCurrents),
                describe(summary.backoffRawCurrents),
                describe(summary.backoffBaselineCurrents),
                describeDelta(summary.backoffFilteredCurrents, summary.backoffBaselineCurrents),
                summary.backoffTargets.isEmpty() ? "[]" : summary.backoffTargets.toString());
    }

    private static String describe(List<Double> values) {
        if (values.isEmpty()) {
            return "[]";
        }
        List<Double> sorted = new ArrayList<>(values);
        Collections.sort(sorted);
        return String.format(
                Locale.US,
                "min=%.2f p10=%.2f p25=%.2f p50=%.2f p75=%.2f p90=%.2f max=%.2f mean=%.2f",
                sorted.get(0),
                percentile(sorted, 0.10),
                percentile(sorted, 0.25),
                percentile(sorted, 0.50),
                percentile(sorted, 0.75),
                percentile(sorted, 0.90),
                sorted.get(sorted.size() - 1),
                sorted.stream().mapToDouble(Double::doubleValue).average().orElse(Double.NaN));
    }

    private static String describeDelta(List<Double> left, List<Double> right) {
        int count = Math.min(left.size(), right.size());
        if (count == 0) {
            return "[]";
        }
        List<Double> deltas = new ArrayList<>(count);
        for (int i = 0; i < count; i++) {
            deltas.add(left.get(i) - right.get(i));
        }
        return describe(deltas);
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

    private static void printThresholdCoverage(
            String label,
            List<Double> activeValues,
            List<Double> backoffValues,
            List<Double> thresholds) {
        System.out.println("  " + label + " threshold coverage");
        for (double threshold : thresholds) {
            long activeCount = activeValues.stream().filter(value -> value >= threshold).count();
            long backoffCount = backoffValues.stream().filter(value -> value >= threshold).count();
            double activeRate = activeValues.isEmpty() ? 0.0 : activeCount * 100.0 / activeValues.size();
            double backoffRate = backoffValues.isEmpty() ? 0.0 : backoffCount * 100.0 / backoffValues.size();
            System.out.printf(
                    Locale.US,
                    "    >= %.1f : active=%d/%d (%.1f%%) backoff=%d/%d (%.1f%%)%n",
                    threshold,
                    activeCount,
                    activeValues.size(),
                    activeRate,
                    backoffCount,
                    backoffValues.size(),
                    backoffRate);
        }
    }

    private record SurveySummary(
            boolean sawNibbleSession,
            boolean truncated,
            double minTargetRot,
            List<Double> activeFilteredCurrents,
            List<Double> activeRawCurrents,
            List<Double> backoffFilteredCurrents,
            List<Double> backoffRawCurrents,
            List<Double> backoffBaselineCurrents,
            List<Double> backoffTargets) {}

    private static final class SurveyState {
        private String sessionMode = "";
        private boolean sessionActive = false;
        private boolean feedLatched = false;
        private boolean nibbleBackoffActive = false;
        private boolean sawNibbleSession = false;
        private boolean truncated = false;
        private double commandedTargetRot = Double.NaN;
        private double minTargetRot = Double.POSITIVE_INFINITY;
        private double rawCurrentAmps = Double.NaN;
        private double filteredCurrentAmps = Double.NaN;
        private double baselineCurrentAmps = Double.NaN;
        private final List<Double> activeFilteredCurrents = new ArrayList<>();
        private final List<Double> activeRawCurrents = new ArrayList<>();
        private final List<Double> backoffFilteredCurrents = new ArrayList<>();
        private final List<Double> backoffRawCurrents = new ArrayList<>();
        private final List<Double> backoffBaselineCurrents = new ArrayList<>();
        private final List<Double> backoffTargets = new ArrayList<>();

        private boolean inActiveNibbleWindow() {
            return sessionActive && feedLatched && "NIBBLE".equals(sessionMode) && !nibbleBackoffActive;
        }
    }

    private static final class EntryIds {
        private int sessionModeEntry = -1;
        private int sessionActiveEntry = -1;
        private int feedLatchedEntry = -1;
        private int commandedTargetEntry = -1;
        private int signalCurrentRawEntry = -1;
        private int signalCurrentFilteredEntry = -1;
        private int signalBaselineEntry = -1;
        private int nibbleBackoffActiveEntry = -1;

        private void capture(int entry, String name) {
            if (name.contains("Intake/SmartRetract/SessionMode")) {
                sessionModeEntry = entry;
            } else if (name.contains("Intake/SmartRetract/SessionActive")) {
                sessionActiveEntry = entry;
            } else if (name.contains("Intake/SmartRetract/FeedLatched")) {
                feedLatchedEntry = entry;
            } else if (name.contains("Intake/SmartRetract/CommandedTargetRot")) {
                commandedTargetEntry = entry;
            } else if (name.contains("Intake/SmartRetract/SignalCurrentRawAmps")) {
                signalCurrentRawEntry = entry;
            } else if (name.contains("Intake/SmartRetract/SignalCurrentFilteredAmps")) {
                signalCurrentFilteredEntry = entry;
            } else if (name.contains("Intake/SmartRetract/SignalBaselineAmps")) {
                signalBaselineEntry = entry;
            } else if (name.contains("Intake/SmartRetract/NibbleBackoffActive")) {
                nibbleBackoffActiveEntry = entry;
            }
        }
    }
}
