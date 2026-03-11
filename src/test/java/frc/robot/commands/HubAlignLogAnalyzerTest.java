package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.Test;

class HubAlignLogAnalyzerTest {
    private static final double LOW_ERROR_WINDOW_DEG = 2.0;
    private static final double LOW_ERROR_OMEGA_THRESHOLD_RAD_PER_SEC = 0.12;

    @Test
    void analyzeHubAlignLogsFromDirectory() throws IOException {
        String directoryProperty = System.getProperty("hubAlignDiag.dir");
        if (directoryProperty == null || directoryProperty.isBlank()) {
            directoryProperty = System.getenv("HUB_ALIGN_DIAG_DIR");
        }
        assumeTrue(
                directoryProperty != null && !directoryProperty.isBlank(),
                "Missing log directory. Set -DhubAlignDiag.dir=<path> or HUB_ALIGN_DIAG_DIR=<path>.");

        Path directory = Path.of(directoryProperty).toAbsolutePath();
        assertTrue(Files.isDirectory(directory), "Directory does not exist: " + directory);

        List<Path> wpilogs = Files.list(directory)
                .filter(path -> path.getFileName().toString().endsWith(".wpilog"))
                .sorted()
                .toList();
        assertTrue(!wpilogs.isEmpty(), "No .wpilog files found in " + directory);

        System.out.println("Hub-align log analysis");
        System.out.println("directory=" + directory);
        for (Path wpilog : wpilogs) {
            System.out.println("analyzing=" + wpilog.getFileName());
            try {
                HubAlignLogSummary summary = analyze(wpilog);
                System.out.println(formatSummary(summary));
            } catch (RuntimeException exception) {
                System.out.println("skipping=" + wpilog.getFileName() + " reason=" + exception.getClass().getSimpleName());
            }
        }
    }

    private static HubAlignLogSummary analyze(Path wpilog) throws IOException {
        DataLogReader reader = new DataLogReader(wpilog.toString());
        if (!reader.isValid()) {
            throw new IllegalStateException("Invalid WPILOG file: " + wpilog);
        }

        EntryIds entryIds = new EntryIds();
        boolean truncated = false;
        Iterator<DataLogRecord> startIterator = reader.iterator();
        while (startIterator.hasNext()) {
            DataLogRecord record;
            try {
                record = startIterator.next();
            } catch (IllegalArgumentException exception) {
                truncated = true;
                break;
            }
            if (!record.isStart()) {
                continue;
            }
            var start = record.getStartData();
            entryIds.capture(start.entry, start.name);
        }

        String gitSha = "";
        String gitDate = "";
        boolean autoAlignRunning = false;
        String mode = "";
        double headingErrorDeg = Double.NaN;
        double omegaCommandRadPerSec = Double.NaN;
        double rawTargetDeg = Double.NaN;
        double effectiveTargetDeg = Double.NaN;
        boolean targetHeld = false;
        boolean usingFallback = false;
        long firstAutoAlignTimestampUs = Long.MIN_VALUE;
        long lastAutoAlignTimestampUs = Long.MIN_VALUE;
        double previousErrorSign = 0.0;
        double previousOmega = Double.NaN;
        long previousOmegaTimestampUs = Long.MIN_VALUE;
        double previousEffectiveTargetDeg = Double.NaN;
        long previousEffectiveTargetTimestampUs = Long.MIN_VALUE;
        int lowErrorSignFlips = 0;
        int omegaZeroCrossingsNearTarget = 0;
        double maxOmegaAccelRadPerSecSq = 0.0;
        double p95OmegaAccelRadPerSecSq = Double.NaN;
        double maxEffectiveTargetStepDeg = 0.0;
        double maxRawTargetStepDeg = 0.0;
        long targetHeldTrueSamples = 0;
        long fallbackTrueSamples = 0;
        List<Double> absErrorSamplesDeg = new ArrayList<>();
        List<Double> omegaAccelSamples = new ArrayList<>();
        List<Double> absOmegaNearTargetSamples = new ArrayList<>();
        int energeticOmegaNearTargetSamples = 0;

        double previousRawTargetDeg = Double.NaN;
        long previousRawTargetTimestampUs = Long.MIN_VALUE;

        Iterator<DataLogRecord> dataIterator = reader.iterator();
        while (dataIterator.hasNext()) {
            DataLogRecord record;
            try {
                record = dataIterator.next();
            } catch (IllegalArgumentException exception) {
                truncated = true;
                break;
            }
            if (record.isStart() || record.isControl()) {
                continue;
            }

            int entry = record.getEntry();
            long timestampUs = record.getTimestamp();

            if (entry == entryIds.gitShaEntry) {
                gitSha = record.getString();
                continue;
            }
            if (entry == entryIds.gitDateEntry) {
                gitDate = record.getString();
                continue;
            }
            if (entry == entryIds.modeEntry) {
                mode = record.getString();
                continue;
            }
            if (entry == entryIds.autoAlignRunningEntry) {
                autoAlignRunning = record.getBoolean();
                continue;
            }
            if (entry == entryIds.headingErrorEntry) {
                headingErrorDeg = record.getDouble();
            } else if (entry == entryIds.omegaCommandEntry) {
                omegaCommandRadPerSec = record.getDouble();
            } else if (entry == entryIds.rawTargetEntry) {
                rawTargetDeg = record.getDouble();
            } else if (entry == entryIds.effectiveTargetEntry) {
                effectiveTargetDeg = record.getDouble();
            } else if (entry == entryIds.targetHeldEntry) {
                targetHeld = record.getBoolean();
            } else if (entry == entryIds.usingFallbackEntry) {
                usingFallback = record.getBoolean();
            } else {
                continue;
            }

            boolean active = autoAlignRunning
                    || Double.isFinite(headingErrorDeg)
                    || Double.isFinite(omegaCommandRadPerSec)
                    || Double.isFinite(effectiveTargetDeg);
            if (!active) {
                continue;
            }

            if (firstAutoAlignTimestampUs == Long.MIN_VALUE) {
                firstAutoAlignTimestampUs = timestampUs;
            }
            lastAutoAlignTimestampUs = timestampUs;

            if (targetHeld) {
                targetHeldTrueSamples++;
            }
            if (usingFallback) {
                fallbackTrueSamples++;
            }

            if (Double.isFinite(headingErrorDeg)) {
                double absErrorDeg = Math.abs(headingErrorDeg);
                absErrorSamplesDeg.add(absErrorDeg);
                if (absErrorDeg <= LOW_ERROR_WINDOW_DEG) {
                    double sign = Math.signum(headingErrorDeg);
                    if (previousErrorSign != 0.0 && sign != 0.0 && sign != previousErrorSign) {
                        lowErrorSignFlips++;
                    }
                    if (sign != 0.0) {
                        previousErrorSign = sign;
                    }
                }
            }

            if (Double.isFinite(omegaCommandRadPerSec)) {
                if (Double.isFinite(headingErrorDeg) && Math.abs(headingErrorDeg) <= LOW_ERROR_WINDOW_DEG) {
                    double absOmega = Math.abs(omegaCommandRadPerSec);
                    absOmegaNearTargetSamples.add(absOmega);
                    if (absOmega >= LOW_ERROR_OMEGA_THRESHOLD_RAD_PER_SEC) {
                        energeticOmegaNearTargetSamples++;
                    }
                }
                if (Double.isFinite(previousOmega) && previousOmegaTimestampUs != Long.MIN_VALUE) {
                    double dtSec = (timestampUs - previousOmegaTimestampUs) / 1_000_000.0;
                    if (dtSec > 1e-6 && dtSec < 0.1) {
                        double accel = Math.abs((omegaCommandRadPerSec - previousOmega) / dtSec);
                        omegaAccelSamples.add(accel);
                        maxOmegaAccelRadPerSecSq = Math.max(maxOmegaAccelRadPerSecSq, accel);
                        if (Double.isFinite(headingErrorDeg)
                                && Math.abs(headingErrorDeg) <= LOW_ERROR_WINDOW_DEG
                                && Math.signum(omegaCommandRadPerSec) != 0.0
                                && Math.signum(previousOmega) != 0.0
                                && Math.signum(omegaCommandRadPerSec) != Math.signum(previousOmega)) {
                            omegaZeroCrossingsNearTarget++;
                        }
                    }
                }
                previousOmega = omegaCommandRadPerSec;
                previousOmegaTimestampUs = timestampUs;
            }

            if (Double.isFinite(effectiveTargetDeg)) {
                if (Double.isFinite(previousEffectiveTargetDeg) && previousEffectiveTargetTimestampUs != Long.MIN_VALUE) {
                    double dtSec = (timestampUs - previousEffectiveTargetTimestampUs) / 1_000_000.0;
                    if (dtSec > 1e-6 && dtSec < 0.1) {
                        maxEffectiveTargetStepDeg = Math.max(
                                maxEffectiveTargetStepDeg,
                                Math.abs(MathUtil.inputModulus(
                                        effectiveTargetDeg - previousEffectiveTargetDeg,
                                        -180.0,
                                        180.0)));
                    }
                }
                previousEffectiveTargetDeg = effectiveTargetDeg;
                previousEffectiveTargetTimestampUs = timestampUs;
            }

            if (Double.isFinite(rawTargetDeg)) {
                if (Double.isFinite(previousRawTargetDeg) && previousRawTargetTimestampUs != Long.MIN_VALUE) {
                    double dtSec = (timestampUs - previousRawTargetTimestampUs) / 1_000_000.0;
                    if (dtSec > 1e-6 && dtSec < 0.1) {
                        maxRawTargetStepDeg = Math.max(
                                maxRawTargetStepDeg,
                                Math.abs(MathUtil.inputModulus(
                                        rawTargetDeg - previousRawTargetDeg,
                                        -180.0,
                                        180.0)));
                    }
                }
                previousRawTargetDeg = rawTargetDeg;
                previousRawTargetTimestampUs = timestampUs;
            }
        }

        p95OmegaAccelRadPerSecSq = percentile(omegaAccelSamples, 0.95);

        double activeDurationSec = Double.NaN;
        if (firstAutoAlignTimestampUs != Long.MIN_VALUE && lastAutoAlignTimestampUs != Long.MIN_VALUE) {
            activeDurationSec = (lastAutoAlignTimestampUs - firstAutoAlignTimestampUs) / 1_000_000.0;
        }

        return new HubAlignLogSummary(
                wpilog,
                gitSha,
                gitDate,
                mode,
                absErrorSamplesDeg.size(),
                activeDurationSec,
                percentile(absErrorSamplesDeg, 0.50),
                percentile(absErrorSamplesDeg, 0.90),
                percentile(absErrorSamplesDeg, 0.95),
                lowErrorSignFlips,
                omegaZeroCrossingsNearTarget,
                maxOmegaAccelRadPerSecSq,
                p95OmegaAccelRadPerSecSq,
                maxEffectiveTargetStepDeg,
                maxRawTargetStepDeg,
                targetHeldTrueSamples,
                fallbackTrueSamples,
                percentile(absOmegaNearTargetSamples, 0.90),
                percentile(absOmegaNearTargetSamples, 0.95),
                energeticOmegaNearTargetSamples,
                entryIds.describeTelemetryVersion(truncated));
    }

    private static double percentile(List<Double> samples, double percentile) {
        if (samples.isEmpty()) {
            return Double.NaN;
        }
        List<Double> sorted = new ArrayList<>(samples);
        sorted.sort(Comparator.naturalOrder());
        int index = (int) Math.round(MathUtil.clamp(percentile, 0.0, 1.0) * (sorted.size() - 1));
        return sorted.get(index);
    }

    private static String formatSummary(HubAlignLogSummary summary) {
        return String.format(
                Locale.US,
                "%s | sha=%s | date=%s | telemetry=%s | samples=%d | active=%.2fs | p90Err=%.2f | p95Err=%.2f | lowErrFlips=%d | omegaCross=%d | p90LowErrOmega=%.2f | p95LowErrOmega=%.2f | energeticLowErrOmega=%d | maxAccel=%.2f | p95Accel=%.2f | maxTargetStep=%.2f | maxRawStep=%.2f | held=%d | fallback=%d",
                summary.path().getFileName(),
                summary.gitSha().isBlank() ? "<unknown>" : summary.gitSha(),
                summary.gitDate().isBlank() ? "<unknown>" : summary.gitDate(),
                summary.telemetryVersion(),
                summary.sampleCount(),
                summary.activeDurationSec(),
                summary.p90AbsErrorDeg(),
                summary.p95AbsErrorDeg(),
                summary.lowErrorSignFlips(),
                summary.omegaZeroCrossingsNearTarget(),
                summary.p90AbsOmegaNearTargetRadPerSec(),
                summary.p95AbsOmegaNearTargetRadPerSec(),
                summary.energeticOmegaNearTargetSamples(),
                summary.maxOmegaAccelRadPerSecSq(),
                summary.p95OmegaAccelRadPerSecSq(),
                summary.maxEffectiveTargetStepDeg(),
                summary.maxRawTargetStepDeg(),
                summary.targetHeldTrueSamples(),
                summary.fallbackTrueSamples());
    }

    private record HubAlignLogSummary(
            Path path,
            String gitSha,
            String gitDate,
            String mode,
            int sampleCount,
            double activeDurationSec,
            double p50AbsErrorDeg,
            double p90AbsErrorDeg,
            double p95AbsErrorDeg,
            int lowErrorSignFlips,
            int omegaZeroCrossingsNearTarget,
            double maxOmegaAccelRadPerSecSq,
            double p95OmegaAccelRadPerSecSq,
            double maxEffectiveTargetStepDeg,
            double maxRawTargetStepDeg,
            long targetHeldTrueSamples,
            long fallbackTrueSamples,
            double p90AbsOmegaNearTargetRadPerSec,
            double p95AbsOmegaNearTargetRadPerSec,
            int energeticOmegaNearTargetSamples,
            String telemetryVersion) {}

    private static final class EntryIds {
        int headingErrorEntry = -1;
        int omegaCommandEntry = -1;
        int rawTargetEntry = -1;
        int effectiveTargetEntry = -1;
        int targetHeldEntry = -1;
        int usingFallbackEntry = -1;
        int autoAlignRunningEntry = -1;
        int modeEntry = -1;
        int gitShaEntry = -1;
        int gitDateEntry = -1;

        void capture(int entry, String name) {
            if (name.contains("Drive/AutoAlign/HeadingErrorDeg")) {
                headingErrorEntry = entry;
            } else if (name.contains("Drive/AutoAlign/OmegaCommandRadPerSec")) {
                omegaCommandEntry = entry;
            } else if (name.contains("Drive/AutoAlign/RawTargetDeg")) {
                rawTargetEntry = entry;
            } else if (name.contains("Drive/AutoAlign/EffectiveTargetDeg")) {
                effectiveTargetEntry = entry;
            } else if (name.contains("Drive/AutoAlign/TargetHeld")) {
                targetHeldEntry = entry;
            } else if (name.contains("Drive/AutoAlign/UsingFallback")) {
                usingFallbackEntry = entry;
            } else if (name.contains("Commands/byName/DriveAutoAlignToHubPose/running")) {
                autoAlignRunningEntry = entry;
            } else if (name.contains("RobotState/Mode")) {
                modeEntry = entry;
            } else if (name.contains("Git SHA")) {
                gitShaEntry = entry;
            } else if (name.contains("Git Date")) {
                gitDateEntry = entry;
            }
        }

        String describeTelemetryVersion(boolean truncated) {
            List<String> tags = new ArrayList<>();
            if (rawTargetEntry != -1) {
                tags.add("rawTarget");
            }
            if (effectiveTargetEntry != -1) {
                tags.add("effectiveTarget");
            }
            if (targetHeldEntry != -1) {
                tags.add("hold");
            }
            if (usingFallbackEntry != -1) {
                tags.add("fallback");
            }
            if (autoAlignRunningEntry != -1) {
                tags.add("cmd");
            }
            String version = tags.isEmpty() ? "legacy" : String.join("+", tags);
            return truncated ? version + "+truncated" : version;
        }
    }
}
