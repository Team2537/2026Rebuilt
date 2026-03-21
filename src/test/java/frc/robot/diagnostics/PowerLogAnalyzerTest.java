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
import org.junit.jupiter.api.Test;

class PowerLogAnalyzerTest {
    @Test
    void analyzePowerAndBrownoutsFromWpiLog() throws IOException {
        String wpilogPath = System.getProperty("powerDiag.wpilog");
        if (wpilogPath == null || wpilogPath.isBlank()) {
            wpilogPath = System.getenv("POWER_DIAG_WPILOG");
        }
        assumeTrue(
                wpilogPath != null && !wpilogPath.isBlank(),
                "Missing power log path. Set -DpowerDiag.wpilog=<path> or POWER_DIAG_WPILOG=<path>.");

        Path wpilog = Path.of(wpilogPath).toAbsolutePath();
        assertTrue(Files.exists(wpilog), "WPILOG does not exist: " + wpilog);

        String report = analyze(wpilog);

        String outPath = System.getProperty("powerDiag.out");
        if (outPath == null || outPath.isBlank()) {
            outPath = System.getenv("POWER_DIAG_OUT");
        }
        if (outPath != null && !outPath.isBlank()) {
            Path out = Path.of(outPath).toAbsolutePath();
            Files.createDirectories(out.getParent());
            Files.writeString(out, report);
            System.out.println("Power summary written: " + out);
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
                "PowerDistribution/Voltage",
                "PowerDistribution/totalVoltageVolts",
                "RobotState/BatteryVoltage",
                "SystemStats/BatteryVoltage");
        int batteryLowEntry = findEntry(entries, "RobotState/BatteryLow");
        int totalCurrentEntry = findEntry(entries, "PowerDistribution/totalCurrentAmps", "PowerDistribution/TotalCurrent");
        int totalPowerEntry = findEntry(entries, "PowerDistribution/TotalPower", "PowerDistribution/totalPowerWatts");
        int temperatureEntry = findEntry(entries, "PowerDistribution/temperatureCelsius", "PowerDistribution/Temperature");
        int enabledEntry = findEntry(entries, "RobotState/Enabled");
        int modeEntry = findEntry(entries, "RobotState/Mode");
        int matchTimeEntry = findEntry(entries, "RobotState/MatchTime");
        int brownoutEntry = findEntry(entries, "SystemStats/BrownedOut");
        int brownoutVoltageEntry = findEntry(entries, "SystemStats/BrownoutVoltage");

        Stats stats = new Stats();
        stats.discoveredPowerKeys.addAll(discoveredKeys(entries));

        double currentVoltage = Double.NaN;
        double currentCurrent = Double.NaN;
        double currentPower = Double.NaN;
        double currentTemp = Double.NaN;
        boolean enabled = false;
        String mode = "";
        double matchTime = Double.NaN;
        boolean batteryLow = false;
        Boolean brownoutFlag = null;
        long firstTimestampUs = Long.MIN_VALUE;
        long lastTimestampUs = Long.MIN_VALUE;
        long enabledStartUs = Long.MIN_VALUE;
        long lowVoltageStartUs = Long.MIN_VALUE;
        long lowVoltage95StartUs = Long.MIN_VALUE;
        long lowVoltage90StartUs = Long.MIN_VALUE;
        long batteryLowStartUs = Long.MIN_VALUE;
        long brownoutStartUs = Long.MIN_VALUE;

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
            boolean emittedWindowEvent = false;
            boolean relevantPowerSample = false;

            if (entry == batteryVoltageEntry) {
                currentVoltage = record.getDouble();
                relevantPowerSample = true;
                stats.observeVoltage(ts, currentVoltage, enabled, mode, matchTime, currentCurrent, currentPower);

                if (currentVoltage < 10.5 && lowVoltageStartUs == Long.MIN_VALUE) {
                    lowVoltageStartUs = ts;
                } else if (currentVoltage >= 10.5 && lowVoltageStartUs != Long.MIN_VALUE) {
                    stats.lowVoltage105Us += ts - lowVoltageStartUs;
                    stats.lowVoltage105Windows.add(windowSummary(lowVoltageStartUs, ts, "V<10.5", enabled, mode));
                    lowVoltageStartUs = Long.MIN_VALUE;
                }

                if (currentVoltage < 9.5 && lowVoltage95StartUs == Long.MIN_VALUE) {
                    lowVoltage95StartUs = ts;
                } else if (currentVoltage >= 9.5 && lowVoltage95StartUs != Long.MIN_VALUE) {
                    stats.lowVoltage95Us += ts - lowVoltage95StartUs;
                    stats.lowVoltage95Windows.add(windowSummary(lowVoltage95StartUs, ts, "V<9.5", enabled, mode));
                    lowVoltage95StartUs = Long.MIN_VALUE;
                }

                if (currentVoltage < 9.0 && lowVoltage90StartUs == Long.MIN_VALUE) {
                    lowVoltage90StartUs = ts;
                } else if (currentVoltage >= 9.0 && lowVoltage90StartUs != Long.MIN_VALUE) {
                    stats.lowVoltage90Us += ts - lowVoltage90StartUs;
                    stats.lowVoltage90Windows.add(windowSummary(lowVoltage90StartUs, ts, "V<9.0", enabled, mode));
                    lowVoltage90StartUs = Long.MIN_VALUE;
                }
            } else if (entry == totalCurrentEntry) {
                currentCurrent = record.getDouble();
                relevantPowerSample = true;
                stats.observeCurrent(ts, currentCurrent, enabled, mode, matchTime, currentVoltage);
                if (Double.isFinite(currentVoltage)) {
                    stats.observeEstimatedPower(ts, currentVoltage * currentCurrent, enabled, mode, matchTime, currentVoltage, currentCurrent);
                }
            } else if (entry == totalPowerEntry) {
                currentPower = record.getDouble();
                relevantPowerSample = true;
                stats.observePower(ts, currentPower, enabled, mode, matchTime, currentVoltage, currentCurrent);
            } else if (entry == temperatureEntry) {
                currentTemp = record.getDouble();
                stats.maxTemperatureC = Math.max(stats.maxTemperatureC, currentTemp);
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
                    stats.modeEvents.add(String.format(Locale.US, "t=%.3f enabled=%s mode=%s matchTime=%.1f V=%.2f I=%.1f P=%.1f",
                            ts / 1_000_000.0, enabled, mode, matchTime, currentVoltage, currentCurrent, currentPower));
                }
            } else if (entry == modeEntry) {
                String newMode = record.getString();
                if (!newMode.equals(mode)) {
                    mode = newMode;
                    stats.modeEvents.add(String.format(Locale.US, "t=%.3f enabled=%s mode=%s matchTime=%.1f V=%.2f I=%.1f P=%.1f",
                            ts / 1_000_000.0, enabled, mode, matchTime, currentVoltage, currentCurrent, currentPower));
                }
            } else if (entry == matchTimeEntry) {
                matchTime = record.getDouble();
            } else if (entry == batteryLowEntry) {
                boolean newBatteryLow = record.getBoolean();
                if (newBatteryLow != batteryLow) {
                    if (newBatteryLow) {
                        batteryLowStartUs = ts;
                    } else if (batteryLowStartUs != Long.MIN_VALUE) {
                        stats.batteryLowUs += ts - batteryLowStartUs;
                        stats.batteryLowWindows.add(windowSummary(batteryLowStartUs, ts, "BatteryLow", enabled, mode));
                        batteryLowStartUs = Long.MIN_VALUE;
                    }
                    batteryLow = newBatteryLow;
                    stats.batteryLowTransitions.add(String.format(Locale.US,
                            "t=%.3f BatteryLow=%s mode=%s matchTime=%.1f V=%.2f I=%.1f P=%.1f",
                            ts / 1_000_000.0, batteryLow, mode, matchTime, currentVoltage, currentCurrent, currentPower));
                }
            } else if (entry == brownoutEntry) {
                boolean newBrownout = readBooleanLenient(record, entries.get(entry).name);
                if (brownoutFlag == null || newBrownout != brownoutFlag.booleanValue()) {
                    if (newBrownout) {
                        brownoutStartUs = ts;
                    } else if (brownoutStartUs != Long.MIN_VALUE) {
                        stats.brownoutUs += ts - brownoutStartUs;
                        stats.brownoutWindows.add(windowSummary(brownoutStartUs, ts, "BrownoutFlag", enabled, mode));
                        brownoutStartUs = Long.MIN_VALUE;
                    }
                    brownoutFlag = newBrownout;
                    stats.brownoutTransitions.add(String.format(Locale.US,
                            "t=%.3f Brownout=%s key=%s mode=%s matchTime=%.1f V=%.2f I=%.1f P=%.1f",
                            ts / 1_000_000.0, newBrownout, entries.get(entry).name, mode, matchTime, currentVoltage, currentCurrent, currentPower));
                }
            } else if (entry == brownoutVoltageEntry) {
                stats.observeBrownoutVoltage(record.getDouble());
            }

            if (relevantPowerSample) {
                if (enabled && currentVoltage < 10.5 && stats.eventSamples.size() < 80 && !emittedWindowEvent) {
                    stats.eventSamples.add(String.format(Locale.US,
                            "t=%.3f mode=%s matchTime=%.1f V=%.2f I=%.1f P=%.1f temp=%.1f",
                            ts / 1_000_000.0, mode, matchTime, currentVoltage, currentCurrent, currentPower, currentTemp));
                    emittedWindowEvent = true;
                }
            }
        }

        if (enabledStartUs != Long.MIN_VALUE) {
            stats.enabledUs += lastTimestampUs - enabledStartUs;
        }
        if (lowVoltageStartUs != Long.MIN_VALUE) {
            stats.lowVoltage105Us += lastTimestampUs - lowVoltageStartUs;
            stats.lowVoltage105Windows.add(windowSummary(lowVoltageStartUs, lastTimestampUs, "V<10.5", enabled, mode));
        }
        if (lowVoltage95StartUs != Long.MIN_VALUE) {
            stats.lowVoltage95Us += lastTimestampUs - lowVoltage95StartUs;
            stats.lowVoltage95Windows.add(windowSummary(lowVoltage95StartUs, lastTimestampUs, "V<9.5", enabled, mode));
        }
        if (lowVoltage90StartUs != Long.MIN_VALUE) {
            stats.lowVoltage90Us += lastTimestampUs - lowVoltage90StartUs;
            stats.lowVoltage90Windows.add(windowSummary(lowVoltage90StartUs, lastTimestampUs, "V<9.0", enabled, mode));
        }
        if (batteryLowStartUs != Long.MIN_VALUE) {
            stats.batteryLowUs += lastTimestampUs - batteryLowStartUs;
            stats.batteryLowWindows.add(windowSummary(batteryLowStartUs, lastTimestampUs, "BatteryLow", enabled, mode));
        }
        if (brownoutStartUs != Long.MIN_VALUE) {
            stats.brownoutUs += lastTimestampUs - brownoutStartUs;
            stats.brownoutWindows.add(windowSummary(brownoutStartUs, lastTimestampUs, "BrownoutFlag", enabled, mode));
        }

        return stats.format(wpilog, entries, batteryVoltageEntry, totalCurrentEntry, totalPowerEntry, temperatureEntry,
                enabledEntry, modeEntry, matchTimeEntry, batteryLowEntry, brownoutEntry, brownoutVoltageEntry,
                firstTimestampUs, lastTimestampUs);
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
        throw new IllegalStateException("Unable to read brownout flag for entry: " + name);
    }

    private static List<EntryInfo> discoveredKeys(Map<Integer, EntryInfo> entries) {
        List<EntryInfo> results = new ArrayList<>();
        for (EntryInfo info : entries.values()) {
            String lower = info.name.toLowerCase(Locale.ROOT);
            if (lower.contains("power") || lower.contains("voltage") || lower.contains("current")
                    || lower.contains("brown") || lower.contains("battery") || lower.contains("enabled")
                    || lower.contains("matchtime")) {
                results.add(info);
            }
        }
        results.sort(Comparator.comparing(info -> info.name));
        return results;
    }

    private static int findEntry(Map<Integer, EntryInfo> entries, String... suffixes) {
        for (var entry : entries.entrySet()) {
            for (String suffix : suffixes) {
                if (entry.getValue().name.endsWith(suffix)) {
                    return entry.getKey();
                }
            }
        }
        return -1;
    }

    private static int findEntryContains(Map<Integer, EntryInfo> entries, String needle) {
        String lowerNeedle = needle.toLowerCase(Locale.ROOT);
        for (var entry : entries.entrySet()) {
            if (entry.getValue().name.toLowerCase(Locale.ROOT).contains(lowerNeedle)) {
                return entry.getKey();
            }
        }
        return -1;
    }

    private static String windowSummary(long startUs, long endUs, String label, boolean enabled, String mode) {
        return String.format(Locale.US, "%s %.3f-%.3f dur=%.3fs enabled=%s mode=%s",
                label, startUs / 1_000_000.0, endUs / 1_000_000.0, (endUs - startUs) / 1_000_000.0, enabled, mode);
    }

    private record EntryInfo(String name, String type) {}

    private static final class Sample {
        private final long timestampUs;
        private final double value;
        private final boolean enabled;
        private final String mode;
        private final double matchTime;
        private final double voltage;
        private final double current;
        private final double power;

        private Sample(long timestampUs, double value, boolean enabled, String mode, double matchTime,
                double voltage, double current, double power) {
            this.timestampUs = timestampUs;
            this.value = value;
            this.enabled = enabled;
            this.mode = mode;
            this.matchTime = matchTime;
            this.voltage = voltage;
            this.current = current;
            this.power = power;
        }
    }

    private static final class Stats {
        private final List<EntryInfo> discoveredPowerKeys = new ArrayList<>();
        private final List<Sample> voltageMinSamples = new ArrayList<>();
        private final List<Sample> currentMaxSamples = new ArrayList<>();
        private final List<Sample> powerMaxSamples = new ArrayList<>();
        private final List<Sample> estimatedPowerMaxSamples = new ArrayList<>();
        private final List<String> eventSamples = new ArrayList<>();
        private final List<String> batteryLowTransitions = new ArrayList<>();
        private final List<String> brownoutTransitions = new ArrayList<>();
        private final List<String> batteryLowWindows = new ArrayList<>();
        private final List<String> brownoutWindows = new ArrayList<>();
        private final List<String> lowVoltage105Windows = new ArrayList<>();
        private final List<String> lowVoltage95Windows = new ArrayList<>();
        private final List<String> lowVoltage90Windows = new ArrayList<>();
        private final List<String> modeEvents = new ArrayList<>();
        private long enabledUs = 0;
        private long lowVoltage105Us = 0;
        private long lowVoltage95Us = 0;
        private long lowVoltage90Us = 0;
        private long batteryLowUs = 0;
        private long brownoutUs = 0;
        private double minVoltage = Double.POSITIVE_INFINITY;
        private double maxVoltage = Double.NEGATIVE_INFINITY;
        private double sumVoltage = 0.0;
        private long voltageSamples = 0;
        private double enabledMinVoltage = Double.POSITIVE_INFINITY;
        private double enabledSumVoltage = 0.0;
        private long enabledVoltageSamples = 0;
        private double maxCurrent = Double.NEGATIVE_INFINITY;
        private double sumCurrent = 0.0;
        private long currentSamples = 0;
        private double enabledMaxCurrent = Double.NEGATIVE_INFINITY;
        private double enabledSumCurrent = 0.0;
        private long enabledCurrentSamples = 0;
        private double maxPower = Double.NEGATIVE_INFINITY;
        private double sumPower = 0.0;
        private long powerSamples = 0;
        private double enabledMaxPower = Double.NEGATIVE_INFINITY;
        private double enabledSumPower = 0.0;
        private long enabledPowerSamples = 0;
        private double estimatedMaxPower = Double.NEGATIVE_INFINITY;
        private double estimatedSumPower = 0.0;
        private long estimatedPowerSamples = 0;
        private double estimatedEnabledMaxPower = Double.NEGATIVE_INFINITY;
        private double estimatedEnabledSumPower = 0.0;
        private long estimatedEnabledPowerSamples = 0;
        private double maxTemperatureC = Double.NEGATIVE_INFINITY;
        private double minBrownoutVoltage = Double.POSITIVE_INFINITY;
        private double maxBrownoutVoltage = Double.NEGATIVE_INFINITY;
        private final Map<String, Integer> modeCounts = new LinkedHashMap<>();

        private void observeVoltage(long ts, double voltage, boolean enabled, String mode, double matchTime, double current, double power) {
            minVoltage = Math.min(minVoltage, voltage);
            maxVoltage = Math.max(maxVoltage, voltage);
            sumVoltage += voltage;
            voltageSamples++;
            if (enabled) {
                enabledMinVoltage = Math.min(enabledMinVoltage, voltage);
                enabledSumVoltage += voltage;
                enabledVoltageSamples++;
            }
            modeCounts.merge(mode == null || mode.isBlank() ? "<unknown>" : mode, 1, Integer::sum);
            addCandidate(voltageMinSamples, new Sample(ts, voltage, enabled, mode, matchTime, voltage, current, power), true);
        }

        private void observeCurrent(long ts, double current, boolean enabled, String mode, double matchTime, double voltage) {
            maxCurrent = Math.max(maxCurrent, current);
            sumCurrent += current;
            currentSamples++;
            if (enabled) {
                enabledMaxCurrent = Math.max(enabledMaxCurrent, current);
                enabledSumCurrent += current;
                enabledCurrentSamples++;
            }
            addCandidate(currentMaxSamples, new Sample(ts, current, enabled, mode, matchTime, voltage, current, Double.NaN), false);
        }

        private void observePower(long ts, double power, boolean enabled, String mode, double matchTime, double voltage, double current) {
            maxPower = Math.max(maxPower, power);
            sumPower += power;
            powerSamples++;
            if (enabled) {
                enabledMaxPower = Math.max(enabledMaxPower, power);
                enabledSumPower += power;
                enabledPowerSamples++;
            }
            addCandidate(powerMaxSamples, new Sample(ts, power, enabled, mode, matchTime, voltage, current, power), false);
        }

        private void observeEstimatedPower(long ts, double power, boolean enabled, String mode, double matchTime, double voltage, double current) {
            estimatedMaxPower = Math.max(estimatedMaxPower, power);
            estimatedSumPower += power;
            estimatedPowerSamples++;
            if (enabled) {
                estimatedEnabledMaxPower = Math.max(estimatedEnabledMaxPower, power);
                estimatedEnabledSumPower += power;
                estimatedEnabledPowerSamples++;
            }
            addCandidate(estimatedPowerMaxSamples, new Sample(ts, power, enabled, mode, matchTime, voltage, current, power), false);
        }

        private void observeBrownoutVoltage(double voltage) {
            minBrownoutVoltage = Math.min(minBrownoutVoltage, voltage);
            maxBrownoutVoltage = Math.max(maxBrownoutVoltage, voltage);
        }

        private void addCandidate(List<Sample> list, Sample sample, boolean ascending) {
            list.add(sample);
            list.sort((a, b) -> ascending ? Double.compare(a.value, b.value) : Double.compare(b.value, a.value));
            while (list.size() > 12) {
                list.remove(list.size() - 1);
            }
        }

        private String format(Path wpilog, Map<Integer, EntryInfo> entries,
                int batteryVoltageEntry, int totalCurrentEntry, int totalPowerEntry, int temperatureEntry,
                int enabledEntry, int modeEntry, int matchTimeEntry, int batteryLowEntry,
                int brownoutEntry, int brownoutVoltageEntry, long firstTimestampUs, long lastTimestampUs) {
            StringBuilder out = new StringBuilder();
            out.append("Power log summary\n");
            out.append("wpilog=").append(wpilog.toAbsolutePath()).append('\n');
            out.append(String.format(Locale.US, "logSpanSec=%.3f\n", (lastTimestampUs - firstTimestampUs) / 1_000_000.0));
            out.append(String.format(Locale.US, "enabledSec=%.3f\n", enabledUs / 1_000_000.0));
            out.append("entries\n");
            appendEntry(out, "batteryVoltage", entries.get(batteryVoltageEntry));
            appendEntry(out, "batteryLow", entries.get(batteryLowEntry));
            appendEntry(out, "totalCurrent", entries.get(totalCurrentEntry));
            appendEntry(out, "totalPower", entries.get(totalPowerEntry));
            appendEntry(out, "temperature", entries.get(temperatureEntry));
            appendEntry(out, "enabled", entries.get(enabledEntry));
            appendEntry(out, "mode", entries.get(modeEntry));
            appendEntry(out, "matchTime", entries.get(matchTimeEntry));
            appendEntry(out, "brownedOut", brownoutEntry >= 0 ? entries.get(brownoutEntry) : null);
            appendEntry(out, "brownoutVoltage", brownoutVoltageEntry >= 0 ? entries.get(brownoutVoltageEntry) : null);

            out.append("metrics\n");
            out.append(String.format(Locale.US, "  voltage min/avg/max = %.2f / %.2f / %.2f V\n",
                    safe(minVoltage), avg(sumVoltage, voltageSamples), safe(maxVoltage)));
            out.append(String.format(Locale.US, "  enabled voltage min/avg = %.2f / %.2f V\n",
                    safe(enabledMinVoltage), avg(enabledSumVoltage, enabledVoltageSamples)));
            out.append(String.format(Locale.US, "  current max/avg = %.1f / %.1f A\n",
                    safe(maxCurrent), avg(sumCurrent, currentSamples)));
            out.append(String.format(Locale.US, "  enabled current max/avg = %.1f / %.1f A\n",
                    safe(enabledMaxCurrent), avg(enabledSumCurrent, enabledCurrentSamples)));
            out.append(String.format(Locale.US, "  power max/avg = %.1f / %.1f W\n",
                    safe(maxPower), avg(sumPower, powerSamples)));
            out.append(String.format(Locale.US, "  power max/avg = %.1f / %.1f W\n",
                    safe(maxPower), avg(sumPower, powerSamples)));
            out.append(String.format(Locale.US, "  enabled power max/avg = %.1f / %.1f W\n",
                    safe(enabledMaxPower), avg(enabledSumPower, enabledPowerSamples)));
            out.append(String.format(Locale.US, "  estimated power from V*I max/avg = %.1f / %.1f W\n",
                    safe(estimatedMaxPower), avg(estimatedSumPower, estimatedPowerSamples)));
            out.append(String.format(Locale.US, "  estimated enabled power max/avg = %.1f / %.1f W\n",
                    safe(estimatedEnabledMaxPower), avg(estimatedEnabledSumPower, estimatedEnabledPowerSamples)));
            out.append(String.format(Locale.US, "  pdh temp max = %.1f C\n", safe(maxTemperatureC)));
            out.append(String.format(Locale.US, "  brownout voltage threshold min/max = %.2f / %.2f V\n",
                    safe(minBrownoutVoltage), safe(maxBrownoutVoltage)));
            out.append(String.format(Locale.US, "  batteryLow sec = %.3f\n", batteryLowUs / 1_000_000.0));
            out.append(String.format(Locale.US, "  V<10.5 sec = %.3f\n", lowVoltage105Us / 1_000_000.0));
            out.append(String.format(Locale.US, "  V<9.5 sec = %.3f\n", lowVoltage95Us / 1_000_000.0));
            out.append(String.format(Locale.US, "  V<9.0 sec = %.3f\n", lowVoltage90Us / 1_000_000.0));
            out.append(String.format(Locale.US, "  brownoutFlag sec = %.3f\n", brownoutUs / 1_000_000.0));

            out.append("top low-voltage samples\n");
            for (Sample s : voltageMinSamples) {
                out.append(String.format(Locale.US,
                        "  t=%.3f V=%.2f enabled=%s mode=%s matchTime=%.1f I=%.1f P=%.1f\n",
                        s.timestampUs / 1_000_000.0, s.value, s.enabled, s.mode, s.matchTime, s.current, s.power));
            }
            out.append("top current samples\n");
            for (Sample s : currentMaxSamples) {
                out.append(String.format(Locale.US,
                        "  t=%.3f I=%.1f enabled=%s mode=%s matchTime=%.1f V=%.2f\n",
                        s.timestampUs / 1_000_000.0, s.value, s.enabled, s.mode, s.matchTime, s.voltage));
            }
            out.append("top power samples\n");
            for (Sample s : powerMaxSamples) {
                out.append(String.format(Locale.US,
                        "  t=%.3f P=%.1f enabled=%s mode=%s matchTime=%.1f V=%.2f I=%.1f\n",
                        s.timestampUs / 1_000_000.0, s.value, s.enabled, s.mode, s.matchTime, s.voltage, s.current));
            }
            out.append("top estimated V*I power samples\n");
            for (Sample s : estimatedPowerMaxSamples) {
                out.append(String.format(Locale.US,
                        "  t=%.3f P_est=%.1f enabled=%s mode=%s matchTime=%.1f V=%.2f I=%.1f\n",
                        s.timestampUs / 1_000_000.0, s.value, s.enabled, s.mode, s.matchTime, s.voltage, s.current));
            }
            out.append("mode transitions\n");
            for (String line : modeEvents) {
                out.append("  ").append(line).append('\n');
            }
            out.append("batteryLow transitions\n");
            for (String line : batteryLowTransitions) {
                out.append("  ").append(line).append('\n');
            }
            out.append("brownout transitions\n");
            for (String line : brownoutTransitions) {
                out.append("  ").append(line).append('\n');
            }
            out.append("threshold windows\n");
            appendList(out, "  batteryLow", batteryLowWindows);
            appendList(out, "  brownout", brownoutWindows);
            appendList(out, "  V<10.5", lowVoltage105Windows);
            appendList(out, "  V<9.5", lowVoltage95Windows);
            appendList(out, "  V<9.0", lowVoltage90Windows);
            out.append("sampled low-voltage events\n");
            for (String line : eventSamples) {
                out.append("  ").append(line).append('\n');
            }
            out.append("discovered relevant keys\n");
            for (EntryInfo info : discoveredPowerKeys) {
                out.append("  ").append(info.type).append(" | ").append(info.name).append('\n');
            }
            return out.toString();
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

        private void appendList(StringBuilder out, String label, List<String> lines) {
            out.append(label).append('\n');
            if (lines.isEmpty()) {
                out.append("    <none>\n");
                return;
            }
            for (String line : lines) {
                out.append("    ").append(line).append('\n');
            }
        }

        private double avg(double sum, long count) {
            return count == 0 ? Double.NaN : sum / count;
        }

        private double safe(double value) {
            return Double.isFinite(value) ? value : Double.NaN;
        }
    }
}
