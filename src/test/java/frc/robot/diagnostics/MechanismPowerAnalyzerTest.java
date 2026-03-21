package frc.robot.diagnostics;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

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
import java.util.regex.Pattern;
import org.junit.jupiter.api.Test;

class MechanismPowerAnalyzerTest {
    private static final Pattern DRIVE_CURRENT_PATTERN = Pattern.compile(".*/Drive/Module[0-3]/(DriveCurrentAmps|TurnCurrentAmps)$");

    @Test
    void analyzeMechanismCurrentAndVoltage() throws IOException {
        String wpilogPath = System.getProperty("mechPowerDiag.wpilog");
        if (wpilogPath == null || wpilogPath.isBlank()) {
            wpilogPath = System.getenv("MECH_POWER_DIAG_WPILOG");
        }
        assumeTrue(
                wpilogPath != null && !wpilogPath.isBlank(),
                "Missing mechanism power log path. Set -DmechPowerDiag.wpilog=<path> or MECH_POWER_DIAG_WPILOG=<path>.");

        Path wpilog = Path.of(wpilogPath).toAbsolutePath();
        assertTrue(Files.exists(wpilog), "WPILOG does not exist: " + wpilog);

        String report = analyze(wpilog);

        String outPath = System.getProperty("mechPowerDiag.out");
        if (outPath == null || outPath.isBlank()) {
            outPath = System.getenv("MECH_POWER_DIAG_OUT");
        }
        if (outPath != null && !outPath.isBlank()) {
            Path out = Path.of(outPath).toAbsolutePath();
            Files.createDirectories(out.getParent());
            Files.writeString(out, report);
            System.out.println("Mechanism power summary written: " + out);
        }

        System.out.println(report);
    }

    private static String analyze(Path wpilog) throws IOException {
        DataLogReader reader = new DataLogReader(wpilog.toString());
        if (!reader.isValid()) {
            throw new IllegalStateException("Invalid WPILOG file: " + wpilog);
        }

        Map<Integer, EntryInfo> entries = new HashMap<>();
        for (DataLogRecord record : recordsUntilFailure(reader)) {
            if (!record.isStart()) {
                continue;
            }
            var start = record.getStartData();
            entries.put(start.entry, new EntryInfo(start.name, start.type));
        }

        int batteryVoltageEntry = findEntry(entries,
                "SystemStats/BatteryVoltage",
                "PowerDistribution/Voltage",
                "PowerDistribution/totalVoltageVolts",
                "RobotState/BatteryVoltage");
        int batteryLowEntry = findEntry(entries, "RobotState/BatteryLow");
        int brownoutEntry = findEntry(entries, "SystemStats/BrownedOut");
        int brownoutVoltageEntry = findEntry(entries, "SystemStats/BrownoutVoltage");
        int batteryCurrentEntry = findEntry(entries, "SystemStats/BatteryCurrent");
        int enabledEntry = findEntry(entries, "RobotState/Enabled");
        int modeEntry = findEntry(entries, "RobotState/Mode");
        int matchTimeEntry = findEntry(entries, "RobotState/MatchTime");

        Map<Integer, MechanismSignal> mechanismSignals = discoverMechanismSignals(entries);

        Stats stats = new Stats();
        stats.entryCount = mechanismSignals.size();
        stats.mechanismEntries.putAll(mechanismSignals);

        Map<Integer, Double> mechanismCurrents = new HashMap<>();
        double voltage = Double.NaN;
        double batteryCurrent = Double.NaN;
        boolean batteryLow = false;
        boolean brownedOut = false;
        boolean enabled = false;
        String mode = "";
        double matchTime = Double.NaN;
        long firstTimestampUs = Long.MIN_VALUE;
        long lastTimestampUs = Long.MIN_VALUE;
        long lowBatteryStartUs = Long.MIN_VALUE;
        long lowVoltage105StartUs = Long.MIN_VALUE;
        long lowVoltage95StartUs = Long.MIN_VALUE;
        long lowVoltage90StartUs = Long.MIN_VALUE;
        long brownoutStartUs = Long.MIN_VALUE;
        long enabledStartUs = Long.MIN_VALUE;

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

            long ts = record.getTimestamp();
            if (firstTimestampUs == Long.MIN_VALUE) {
                firstTimestampUs = ts;
            }
            lastTimestampUs = ts;

            int entry = record.getEntry();

            if (entry == batteryVoltageEntry) {
                voltage = record.getDouble();
                stats.observeVoltage(ts, voltage, enabled, mode, matchTime, batteryCurrent, sumBreakdown(mechanismCurrents, mechanismSignals));

                if (voltage < 10.5 && lowVoltage105StartUs == Long.MIN_VALUE) {
                    lowVoltage105StartUs = ts;
                } else if (voltage >= 10.5 && lowVoltage105StartUs != Long.MIN_VALUE) {
                    stats.lowVoltage105Us += ts - lowVoltage105StartUs;
                    lowVoltage105StartUs = Long.MIN_VALUE;
                }

                if (voltage < 9.5 && lowVoltage95StartUs == Long.MIN_VALUE) {
                    lowVoltage95StartUs = ts;
                } else if (voltage >= 9.5 && lowVoltage95StartUs != Long.MIN_VALUE) {
                    stats.lowVoltage95Us += ts - lowVoltage95StartUs;
                    lowVoltage95StartUs = Long.MIN_VALUE;
                }

                if (voltage < 9.0 && lowVoltage90StartUs == Long.MIN_VALUE) {
                    lowVoltage90StartUs = ts;
                } else if (voltage >= 9.0 && lowVoltage90StartUs != Long.MIN_VALUE) {
                    stats.lowVoltage90Us += ts - lowVoltage90StartUs;
                    lowVoltage90StartUs = Long.MIN_VALUE;
                }
            } else if (entry == batteryCurrentEntry) {
                batteryCurrent = record.getDouble();
                Breakdown breakdown = sumBreakdown(mechanismCurrents, mechanismSignals);
                stats.observeBatteryCurrent(ts, batteryCurrent, enabled, mode, matchTime, voltage, breakdown);
            } else if (entry == batteryLowEntry) {
                boolean newBatteryLow = record.getBoolean();
                if (newBatteryLow != batteryLow) {
                    if (newBatteryLow) {
                        lowBatteryStartUs = ts;
                    } else if (lowBatteryStartUs != Long.MIN_VALUE) {
                        stats.batteryLowUs += ts - lowBatteryStartUs;
                        lowBatteryStartUs = Long.MIN_VALUE;
                    }
                    batteryLow = newBatteryLow;
                    Breakdown breakdown = sumBreakdown(mechanismCurrents, mechanismSignals);
                    stats.batteryLowEvents.add(formatSnapshot("BatteryLow=" + batteryLow, ts, enabled, mode, matchTime, voltage, batteryCurrent, breakdown));
                }
            } else if (entry == brownoutEntry) {
                boolean newBrownedOut = readBooleanLenient(record, entries.get(entry).name);
                if (newBrownedOut != brownedOut) {
                    if (newBrownedOut) {
                        brownoutStartUs = ts;
                    } else if (brownoutStartUs != Long.MIN_VALUE) {
                        stats.brownoutUs += ts - brownoutStartUs;
                        lowBatteryStartUs = lowBatteryStartUs;
                        brownoutStartUs = Long.MIN_VALUE;
                    }
                    brownedOut = newBrownedOut;
                    Breakdown breakdown = sumBreakdown(mechanismCurrents, mechanismSignals);
                    stats.brownoutEvents.add(formatSnapshot("BrownedOut=" + brownedOut, ts, enabled, mode, matchTime, voltage, batteryCurrent, breakdown));
                }
            } else if (entry == brownoutVoltageEntry) {
                stats.observeBrownoutVoltage(record.getDouble());
            } else if (entry == enabledEntry) {
                boolean newEnabled = record.getBoolean();
                if (newEnabled != enabled) {
                    if (newEnabled) {
                        enabledStartUs = ts;
                    } else if (enabledStartUs != Long.MIN_VALUE) {
                        stats.enabledUs += ts - enabledStartUs;
                        enabledStartUs = Long.MIN_VALUE;
                    }
                    enabled = newEnabled;
                    Breakdown breakdown = sumBreakdown(mechanismCurrents, mechanismSignals);
                    stats.modeEvents.add(formatSnapshot("enabled=" + enabled, ts, enabled, mode, matchTime, voltage, batteryCurrent, breakdown));
                }
            } else if (entry == modeEntry) {
                String newMode = record.getString();
                if (!newMode.equals(mode)) {
                    mode = newMode;
                    Breakdown breakdown = sumBreakdown(mechanismCurrents, mechanismSignals);
                    stats.modeEvents.add(formatSnapshot("mode=" + mode, ts, enabled, mode, matchTime, voltage, batteryCurrent, breakdown));
                }
            } else if (entry == matchTimeEntry) {
                matchTime = record.getDouble();
            } else if (mechanismSignals.containsKey(entry)) {
                mechanismCurrents.put(entry, record.getDouble());
                Breakdown breakdown = sumBreakdown(mechanismCurrents, mechanismSignals);
                stats.observeMechanismCurrent(ts, enabled, mode, matchTime, voltage, batteryCurrent, breakdown);
            }
        }

        if (enabledStartUs != Long.MIN_VALUE) {
            stats.enabledUs += lastTimestampUs - enabledStartUs;
        }
        if (lowBatteryStartUs != Long.MIN_VALUE) {
            stats.batteryLowUs += lastTimestampUs - lowBatteryStartUs;
        }
        if (lowVoltage105StartUs != Long.MIN_VALUE) {
            stats.lowVoltage105Us += lastTimestampUs - lowVoltage105StartUs;
        }
        if (lowVoltage95StartUs != Long.MIN_VALUE) {
            stats.lowVoltage95Us += lastTimestampUs - lowVoltage95StartUs;
        }
        if (lowVoltage90StartUs != Long.MIN_VALUE) {
            stats.lowVoltage90Us += lastTimestampUs - lowVoltage90StartUs;
        }
        if (brownoutStartUs != Long.MIN_VALUE) {
            stats.brownoutUs += lastTimestampUs - brownoutStartUs;
        }

        return stats.format(wpilog, entries, batteryVoltageEntry, batteryCurrentEntry, batteryLowEntry, brownoutEntry,
                brownoutVoltageEntry, enabledEntry, modeEntry, matchTimeEntry, firstTimestampUs, lastTimestampUs);
    }

    private static Map<Integer, MechanismSignal> discoverMechanismSignals(Map<Integer, EntryInfo> entries) {
        Map<Integer, MechanismSignal> results = new LinkedHashMap<>();
        for (var entry : entries.entrySet()) {
            String name = entry.getValue().name;
            if (DRIVE_CURRENT_PATTERN.matcher(name).matches()) {
                results.put(entry.getKey(), new MechanismSignal(name, "drive"));
            } else if (name.endsWith("/Intake/LeftSupplyCurrentAmps")
                    || name.endsWith("/Intake/RightSupplyCurrentAmps")
                    || name.endsWith("/Intake/RollerSupplyCurrentAmps")) {
                results.put(entry.getKey(), new MechanismSignal(name, "intake"));
            } else if (name.endsWith("/Shooter/HoodSupplyCurrentAmps")
                    || name.endsWith("/Shooter/KickerSupplyCurrentAmps")
                    || name.endsWith("/Shooter/ShooterLeftSupplyCurrentAmps")
                    || name.endsWith("/Shooter/ShooterRightSupplyCurrentAmps")) {
                results.put(entry.getKey(), new MechanismSignal(name, "shooter"));
            } else if (name.endsWith("/Transfer/SupplyCurrentAmps")) {
                results.put(entry.getKey(), new MechanismSignal(name, "transfer"));
            }
        }
        return results;
    }

    private static Breakdown sumBreakdown(Map<Integer, Double> currentValues, Map<Integer, MechanismSignal> signals) {
        double drive = 0.0;
        double intake = 0.0;
        double shooter = 0.0;
        double transfer = 0.0;
        for (var entry : currentValues.entrySet()) {
            MechanismSignal signal = signals.get(entry.getKey());
            if (signal == null) {
                continue;
            }
            double value = Math.abs(entry.getValue());
            switch (signal.subsystem) {
                case "drive" -> drive += value;
                case "intake" -> intake += value;
                case "shooter" -> shooter += value;
                case "transfer" -> transfer += value;
                default -> {}
            }
        }
        return new Breakdown(drive, intake, shooter, transfer);
    }

    private static List<DataLogRecord> recordsUntilFailure(DataLogReader reader) {
        List<DataLogRecord> records = new ArrayList<>();
        Iterator<DataLogRecord> iterator = reader.iterator();
        while (true) {
            try {
                if (!iterator.hasNext()) {
                    break;
                }
                records.add(iterator.next());
            } catch (IllegalArgumentException ignored) {
                break;
            }
        }
        return records;
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

    private static String formatSnapshot(String label, long ts, boolean enabled, String mode, double matchTime,
            double voltage, double batteryCurrent, Breakdown breakdown) {
        return String.format(Locale.US,
                "%s t=%.3f enabled=%s mode=%s matchTime=%.1f V=%.2f battI=%.1f mechI=%.1f drive=%.1f intake=%.1f shooter=%.1f transfer=%.1f",
                label,
                ts / 1_000_000.0,
                enabled,
                mode,
                matchTime,
                voltage,
                batteryCurrent,
                breakdown.total(),
                breakdown.drive,
                breakdown.intake,
                breakdown.shooter,
                breakdown.transfer);
    }

    private record EntryInfo(String name, String type) {}

    private record MechanismSignal(String name, String subsystem) {}

    private record Breakdown(double drive, double intake, double shooter, double transfer) {
        private double total() {
            return drive + intake + shooter + transfer;
        }
    }

    private record Snapshot(long timestampUs, boolean enabled, String mode, double matchTime,
            double voltage, double batteryCurrent, Breakdown breakdown) {}

    private static final class Stats {
        private int entryCount = 0;
        private final Map<Integer, MechanismSignal> mechanismEntries = new LinkedHashMap<>();
        private final List<Snapshot> topMechanismCurrentSamples = new ArrayList<>();
        private final List<Snapshot> topBatteryCurrentSamples = new ArrayList<>();
        private final List<Snapshot> lowestVoltageSamples = new ArrayList<>();
        private final List<String> batteryLowEvents = new ArrayList<>();
        private final List<String> brownoutEvents = new ArrayList<>();
        private final List<String> modeEvents = new ArrayList<>();
        private long enabledUs = 0;
        private long batteryLowUs = 0;
        private long brownoutUs = 0;
        private long lowVoltage105Us = 0;
        private long lowVoltage95Us = 0;
        private long lowVoltage90Us = 0;
        private double minVoltage = Double.POSITIVE_INFINITY;
        private double maxVoltage = Double.NEGATIVE_INFINITY;
        private double enabledMinVoltage = Double.POSITIVE_INFINITY;
        private double sumVoltage = 0.0;
        private long voltageSamples = 0;
        private double maxBatteryCurrent = Double.NEGATIVE_INFINITY;
        private double enabledMaxBatteryCurrent = Double.NEGATIVE_INFINITY;
        private double maxMechanismCurrent = Double.NEGATIVE_INFINITY;
        private double enabledMaxMechanismCurrent = Double.NEGATIVE_INFINITY;
        private double maxDriveCurrent = Double.NEGATIVE_INFINITY;
        private double maxIntakeCurrent = Double.NEGATIVE_INFINITY;
        private double maxShooterCurrent = Double.NEGATIVE_INFINITY;
        private double maxTransferCurrent = Double.NEGATIVE_INFINITY;
        private Snapshot peakMechanismSnapshot = null;
        private Snapshot peakDriveSnapshot = null;
        private Snapshot peakIntakeSnapshot = null;
        private Snapshot peakShooterSnapshot = null;
        private Snapshot peakTransferSnapshot = null;
        private double minBrownoutVoltage = Double.POSITIVE_INFINITY;
        private double maxBrownoutVoltage = Double.NEGATIVE_INFINITY;

        private void observeVoltage(long ts, double voltage, boolean enabled, String mode, double matchTime,
                double batteryCurrent, Breakdown breakdown) {
            minVoltage = Math.min(minVoltage, voltage);
            maxVoltage = Math.max(maxVoltage, voltage);
            sumVoltage += voltage;
            voltageSamples++;
            if (enabled) {
                enabledMinVoltage = Math.min(enabledMinVoltage, voltage);
            }
            addLowestVoltage(new Snapshot(ts, enabled, mode, matchTime, voltage, batteryCurrent, breakdown));
        }

        private void observeBatteryCurrent(long ts, double batteryCurrent, boolean enabled, String mode, double matchTime,
                double voltage, Breakdown breakdown) {
            maxBatteryCurrent = Math.max(maxBatteryCurrent, batteryCurrent);
            if (enabled) {
                enabledMaxBatteryCurrent = Math.max(enabledMaxBatteryCurrent, batteryCurrent);
            }
            addTopBatteryCurrent(new Snapshot(ts, enabled, mode, matchTime, voltage, batteryCurrent, breakdown));
        }

        private void observeMechanismCurrent(long ts, boolean enabled, String mode, double matchTime,
                double voltage, double batteryCurrent, Breakdown breakdown) {
            Snapshot snapshot = new Snapshot(ts, enabled, mode, matchTime, voltage, batteryCurrent, breakdown);
            if (breakdown.total() > maxMechanismCurrent) {
                maxMechanismCurrent = breakdown.total();
                peakMechanismSnapshot = snapshot;
            }
            if (enabled) {
                enabledMaxMechanismCurrent = Math.max(enabledMaxMechanismCurrent, breakdown.total());
            }
            if (breakdown.drive() > maxDriveCurrent) {
                maxDriveCurrent = breakdown.drive();
                peakDriveSnapshot = snapshot;
            }
            if (breakdown.intake() > maxIntakeCurrent) {
                maxIntakeCurrent = breakdown.intake();
                peakIntakeSnapshot = snapshot;
            }
            if (breakdown.shooter() > maxShooterCurrent) {
                maxShooterCurrent = breakdown.shooter();
                peakShooterSnapshot = snapshot;
            }
            if (breakdown.transfer() > maxTransferCurrent) {
                maxTransferCurrent = breakdown.transfer();
                peakTransferSnapshot = snapshot;
            }
            addTopMechanismCurrent(snapshot);
        }

        private void observeBrownoutVoltage(double voltage) {
            minBrownoutVoltage = Math.min(minBrownoutVoltage, voltage);
            maxBrownoutVoltage = Math.max(maxBrownoutVoltage, voltage);
        }

        private void addTopMechanismCurrent(Snapshot snapshot) {
            topMechanismCurrentSamples.add(snapshot);
            topMechanismCurrentSamples.sort((a, b) -> Double.compare(b.breakdown.total(), a.breakdown.total()));
            while (topMechanismCurrentSamples.size() > 12) {
                topMechanismCurrentSamples.remove(topMechanismCurrentSamples.size() - 1);
            }
        }

        private void addTopBatteryCurrent(Snapshot snapshot) {
            topBatteryCurrentSamples.add(snapshot);
            topBatteryCurrentSamples.sort((a, b) -> Double.compare(b.batteryCurrent, a.batteryCurrent));
            while (topBatteryCurrentSamples.size() > 12) {
                topBatteryCurrentSamples.remove(topBatteryCurrentSamples.size() - 1);
            }
        }

        private void addLowestVoltage(Snapshot snapshot) {
            lowestVoltageSamples.add(snapshot);
            lowestVoltageSamples.sort(Comparator.comparingDouble(a -> a.voltage));
            while (lowestVoltageSamples.size() > 12) {
                lowestVoltageSamples.remove(lowestVoltageSamples.size() - 1);
            }
        }

        private String format(Path wpilog, Map<Integer, EntryInfo> entries,
                int batteryVoltageEntry, int batteryCurrentEntry, int batteryLowEntry, int brownoutEntry,
                int brownoutVoltageEntry, int enabledEntry, int modeEntry, int matchTimeEntry,
                long firstTimestampUs, long lastTimestampUs) {
            StringBuilder out = new StringBuilder();
            out.append("Mechanism power summary\n");
            out.append("wpilog=").append(wpilog.toAbsolutePath()).append('\n');
            out.append(String.format(Locale.US, "logSpanSec=%.3f\n", (lastTimestampUs - firstTimestampUs) / 1_000_000.0));
            out.append(String.format(Locale.US, "enabledSec=%.3f\n", enabledUs / 1_000_000.0));
            out.append(String.format(Locale.US, "mechanismCurrentEntryCount=%d\n", entryCount));
            out.append("entries\n");
            appendEntry(out, "batteryVoltage", entries.get(batteryVoltageEntry));
            appendEntry(out, "batteryCurrent", entries.get(batteryCurrentEntry));
            appendEntry(out, "batteryLow", entries.get(batteryLowEntry));
            appendEntry(out, "brownedOut", entries.get(brownoutEntry));
            appendEntry(out, "brownoutVoltage", entries.get(brownoutVoltageEntry));
            appendEntry(out, "enabled", entries.get(enabledEntry));
            appendEntry(out, "mode", entries.get(modeEntry));
            appendEntry(out, "matchTime", entries.get(matchTimeEntry));

            out.append("metrics\n");
            out.append(String.format(Locale.US, "  voltage min/avg/max = %.2f / %.2f / %.2f V\n",
                    safe(minVoltage), avg(sumVoltage, voltageSamples), safe(maxVoltage)));
            out.append(String.format(Locale.US, "  enabled voltage min = %.2f V\n", safe(enabledMinVoltage)));
            out.append(String.format(Locale.US, "  max battery current = %.1f A (enabled %.1f A)\n",
                    safe(maxBatteryCurrent), safe(enabledMaxBatteryCurrent)));
            out.append(String.format(Locale.US, "  max summed mechanism current = %.1f A (enabled %.1f A)\n",
                    safe(maxMechanismCurrent), safe(enabledMaxMechanismCurrent)));
            out.append(String.format(Locale.US, "  subsystem peaks: drive=%.1fA intake=%.1fA shooter=%.1fA transfer=%.1fA\n",
                    safe(maxDriveCurrent), safe(maxIntakeCurrent), safe(maxShooterCurrent), safe(maxTransferCurrent)));
            out.append(String.format(Locale.US, "  batteryLow sec = %.3f\n", batteryLowUs / 1_000_000.0));
            out.append(String.format(Locale.US, "  V<10.5 sec = %.3f\n", lowVoltage105Us / 1_000_000.0));
            out.append(String.format(Locale.US, "  V<9.5 sec = %.3f\n", lowVoltage95Us / 1_000_000.0));
            out.append(String.format(Locale.US, "  V<9.0 sec = %.3f\n", lowVoltage90Us / 1_000_000.0));
            out.append(String.format(Locale.US, "  brownoutFlag sec = %.3f\n", brownoutUs / 1_000_000.0));
            out.append(String.format(Locale.US, "  rio brownout threshold = %.2f V\n", safe(minBrownoutVoltage)));

            out.append("peak snapshots\n");
            appendPeak(out, "  overall peak", peakMechanismSnapshot);
            appendPeak(out, "  drive peak", peakDriveSnapshot);
            appendPeak(out, "  intake peak", peakIntakeSnapshot);
            appendPeak(out, "  shooter peak", peakShooterSnapshot);
            appendPeak(out, "  transfer peak", peakTransferSnapshot);
            out.append("top summed mechanism-current samples\n");
            appendSnapshots(out, topMechanismCurrentSamples, true);
            out.append("top battery-current samples\n");
            appendSnapshots(out, topBatteryCurrentSamples, false);
            out.append("lowest-voltage samples\n");
            appendSnapshots(out, lowestVoltageSamples, true);
            out.append("batteryLow transitions\n");
            appendLines(out, batteryLowEvents);
            out.append("brownout transitions\n");
            appendLines(out, brownoutEvents);
            out.append("mode transitions\n");
            appendLines(out, modeEvents);
            out.append("mechanism current entries\n");
            for (MechanismSignal signal : mechanismEntries.values()) {
                out.append("  ").append(signal.subsystem).append(" | ").append(signal.name).append('\n');
            }
            return out.toString();
        }

        private void appendPeak(StringBuilder out, String label, Snapshot s) {
            out.append(label);
            if (s == null) {
                out.append(" <none>\n");
                return;
            }
            out.append(String.format(Locale.US,
                    " t=%.3f enabled=%s mode=%s matchTime=%.1f V=%.2f battI=%.1f mechI=%.1f drive=%.1f intake=%.1f shooter=%.1f transfer=%.1f\n",
                    s.timestampUs / 1_000_000.0,
                    s.enabled,
                    s.mode,
                    s.matchTime,
                    s.voltage,
                    s.batteryCurrent,
                    s.breakdown.total(),
                    s.breakdown.drive(),
                    s.breakdown.intake(),
                    s.breakdown.shooter(),
                    s.breakdown.transfer()));
        }

        private void appendSnapshots(StringBuilder out, List<Snapshot> snapshots, boolean includeMechanism) {
            if (snapshots.isEmpty()) {
                out.append("  <none>\n");
                return;
            }
            for (Snapshot s : snapshots) {
                out.append(String.format(Locale.US,
                        "  t=%.3f enabled=%s mode=%s matchTime=%.1f V=%.2f battI=%.1f mechI=%.1f drive=%.1f intake=%.1f shooter=%.1f transfer=%.1f\n",
                        s.timestampUs / 1_000_000.0,
                        s.enabled,
                        s.mode,
                        s.matchTime,
                        s.voltage,
                        s.batteryCurrent,
                        s.breakdown.total(),
                        s.breakdown.drive(),
                        s.breakdown.intake(),
                        s.breakdown.shooter(),
                        s.breakdown.transfer()));
            }
        }

        private void appendLines(StringBuilder out, List<String> lines) {
            if (lines.isEmpty()) {
                out.append("  <none>\n");
                return;
            }
            for (String line : lines) {
                out.append("  ").append(line).append('\n');
            }
        }

        private void appendEntry(StringBuilder out, String label, EntryInfo info) {
            out.append("  ").append(label).append(" = ");
            if (info == null) {
                out.append("<missing>");
            } else {
                out.append(info.type).append(" | ").append(info.name);
            }
            out.append('\n');
        }

        private double avg(double sum, long count) {
            return count == 0 ? Double.NaN : sum / count;
        }

        private double safe(double value) {
            return Double.isFinite(value) ? value : Double.NaN;
        }
    }
}
