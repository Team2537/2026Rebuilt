package frc.robot.diagnostics;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import org.junit.jupiter.api.Test;

class StutterLogAnalyzerTest {
    @Test
    void analyzeStutterSignals() throws IOException {
        String wpilogPath = System.getProperty("stutterDiag.wpilog");
        if (wpilogPath == null || wpilogPath.isBlank()) {
            wpilogPath = System.getenv("STUTTER_DIAG_WPILOG");
        }
        assumeTrue(
                wpilogPath != null && !wpilogPath.isBlank(),
                "Missing stutter log path. Set -DstutterDiag.wpilog=<path> or STUTTER_DIAG_WPILOG=<path>.");

        Path wpilog = Path.of(wpilogPath).toAbsolutePath();
        assertTrue(Files.exists(wpilog), "WPILOG does not exist: " + wpilog);

        String report = analyze(wpilog);

        String outPath = System.getProperty("stutterDiag.out");
        if (outPath == null || outPath.isBlank()) {
            outPath = System.getenv("STUTTER_DIAG_OUT");
        }
        if (outPath != null && !outPath.isBlank()) {
            Path out = Path.of(outPath).toAbsolutePath();
            Files.createDirectories(out.getParent());
            Files.writeString(out, report);
            System.out.println("Stutter summary written: " + out);
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

        int voltageEntry = findEntry(entries, "SystemStats/BatteryVoltage", "PowerDistribution/Voltage", "RobotState/BatteryVoltage");
        int batteryLowEntry = findEntry(entries, "RobotState/BatteryLow");
        int brownoutEntry = findEntry(entries, "SystemStats/BrownedOut");
        int enabledEntry = findEntry(entries, "RobotState/Enabled");
        int modeEntry = findEntry(entries, "RobotState/Mode");
        int matchTimeEntry = findEntry(entries, "RobotState/MatchTime");
        int gyroConnectedEntry = findEntry(entries, "RobotState/GyroConnected");
        int visionConnectedEntry = findEntry(entries, "RobotState/VisionConnected");
        int cam0ConnectedEntry = findEntry(entries, "RobotState/Vision/Camera0Connected");
        int cam1ConnectedEntry = findEntry(entries, "RobotState/Vision/Camera1Connected");
        int radioConnectedEntry = findEntry(entries, "RadioStatus/Connected");
        int[] moduleDriveConnectedEntries = new int[] {
                findEntry(entries, "Drive/Module0/DriveConnected"),
                findEntry(entries, "Drive/Module1/DriveConnected"),
                findEntry(entries, "Drive/Module2/DriveConnected"),
                findEntry(entries, "Drive/Module3/DriveConnected")
        };
        int[] moduleTurnConnectedEntries = new int[] {
                findEntry(entries, "Drive/Module0/TurnConnected"),
                findEntry(entries, "Drive/Module1/TurnConnected"),
                findEntry(entries, "Drive/Module2/TurnConnected"),
                findEntry(entries, "Drive/Module3/TurnConnected")
        };
        int[] moduleEncoderConnectedEntries = new int[] {
                findEntry(entries, "Drive/Module0/TurnEncoderConnected"),
                findEntry(entries, "Drive/Module1/TurnEncoderConnected"),
                findEntry(entries, "Drive/Module2/TurnEncoderConnected"),
                findEntry(entries, "Drive/Module3/TurnEncoderConnected")
        };
        int joystickAxesEntry = findEntry(entries, "DriverStation/Joystick0/AxisValues");
        int joystickButtonsEntry = findEntry(entries, "DriverStation/Joystick0/ButtonValues");
        int joystickPovsEntry = findEntry(entries, "DriverStation/Joystick0/POVs");
        int reqSpeedsEntry = findEntry(entries, "SwerveChassisSpeeds/Requested");
        int measSpeedsEntry = findEntry(entries, "SwerveChassisSpeeds/Measured");

        Stats stats = new Stats();
        stats.entries.put("voltage", entries.get(voltageEntry));
        stats.entries.put("batteryLow", entries.get(batteryLowEntry));
        stats.entries.put("brownout", entries.get(brownoutEntry));
        stats.entries.put("enabled", entries.get(enabledEntry));
        stats.entries.put("mode", entries.get(modeEntry));
        stats.entries.put("matchTime", entries.get(matchTimeEntry));
        stats.entries.put("gyroConnected", entries.get(gyroConnectedEntry));
        stats.entries.put("visionConnected", entries.get(visionConnectedEntry));
        stats.entries.put("cam0Connected", entries.get(cam0ConnectedEntry));
        stats.entries.put("cam1Connected", entries.get(cam1ConnectedEntry));
        stats.entries.put("radioConnected", entries.get(radioConnectedEntry));
        for (int i = 0; i < 4; i++) {
            stats.entries.put("module" + i + "DriveConnected", entries.get(moduleDriveConnectedEntries[i]));
            stats.entries.put("module" + i + "TurnConnected", entries.get(moduleTurnConnectedEntries[i]));
            stats.entries.put("module" + i + "EncoderConnected", entries.get(moduleEncoderConnectedEntries[i]));
        }
        stats.entries.put("joystickAxes", entries.get(joystickAxesEntry));
        stats.entries.put("joystickButtons", entries.get(joystickButtonsEntry));
        stats.entries.put("joystickPovs", entries.get(joystickPovsEntry));
        stats.entries.put("requestedSpeeds", entries.get(reqSpeedsEntry));
        stats.entries.put("measuredSpeeds", entries.get(measSpeedsEntry));

        boolean enabled = false;
        String mode = "";
        double matchTime = Double.NaN;
        double voltage = Double.NaN;
        boolean batteryLow = false;
        boolean brownout = false;
        boolean gyroConnected = true;
        boolean visionConnected = true;
        boolean cam0Connected = true;
        boolean cam1Connected = true;
        Boolean radioConnected = null;
        boolean[] moduleDriveConnected = new boolean[] { true, true, true, true };
        boolean[] moduleTurnConnected = new boolean[] { true, true, true, true };
        boolean[] moduleEncoderConnected = new boolean[] { true, true, true, true };
        double[] joystickAxes = null;
        long joystickButtons = 0L;
        long[] joystickPovs = null;
        ChassisSample requested = null;
        ChassisSample measured = null;

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
            int entry = record.getEntry();

            if (entry == enabledEntry) {
                boolean newValue = record.getBoolean();
                if (newValue != enabled) {
                    enabled = newValue;
                    stats.events.add(snapshot("enabled=" + enabled, ts, mode, matchTime, voltage, batteryLow, brownout,
                            gyroConnected, visionConnected, cam0Connected, cam1Connected, radioConnected,
                            joystickAxes, joystickButtons, requested, measured));
                }
            } else if (entry == modeEntry) {
                String newValue = record.getString();
                if (!newValue.equals(mode)) {
                    mode = newValue;
                    stats.events.add(snapshot("mode=" + mode, ts, mode, matchTime, voltage, batteryLow, brownout,
                            gyroConnected, visionConnected, cam0Connected, cam1Connected, radioConnected,
                            joystickAxes, joystickButtons, requested, measured));
                }
            } else if (entry == matchTimeEntry) {
                matchTime = record.getDouble();
            } else if (entry == voltageEntry) {
                voltage = record.getDouble();
                if (enabled && voltage < 9.0 && stats.lowVoltageSnapshots.size() < 40) {
                    stats.lowVoltageSnapshots.add(snapshot("lowVoltage", ts, mode, matchTime, voltage, batteryLow, brownout,
                            gyroConnected, visionConnected, cam0Connected, cam1Connected, radioConnected,
                            joystickAxes, joystickButtons, requested, measured));
                }
            } else if (entry == batteryLowEntry) {
                boolean newValue = record.getBoolean();
                if (newValue != batteryLow) {
                    batteryLow = newValue;
                    stats.events.add(snapshot("batteryLow=" + batteryLow, ts, mode, matchTime, voltage, batteryLow, brownout,
                            gyroConnected, visionConnected, cam0Connected, cam1Connected, radioConnected,
                            joystickAxes, joystickButtons, requested, measured));
                }
            } else if (entry == brownoutEntry) {
                boolean newValue = readBooleanLenient(record, entries.get(entry).name);
                if (newValue != brownout) {
                    brownout = newValue;
                    stats.events.add(snapshot("brownout=" + brownout, ts, mode, matchTime, voltage, batteryLow, brownout,
                            gyroConnected, visionConnected, cam0Connected, cam1Connected, radioConnected,
                            joystickAxes, joystickButtons, requested, measured));
                }
            } else if (entry == gyroConnectedEntry) {
                boolean newValue = record.getBoolean();
                if (newValue != gyroConnected) {
                    gyroConnected = newValue;
                    stats.disconnectEvents.add(snapshot("gyroConnected=" + gyroConnected, ts, mode, matchTime, voltage, batteryLow, brownout,
                            gyroConnected, visionConnected, cam0Connected, cam1Connected, radioConnected,
                            joystickAxes, joystickButtons, requested, measured));
                }
            } else if (entry == visionConnectedEntry) {
                boolean newValue = record.getBoolean();
                if (newValue != visionConnected) {
                    visionConnected = newValue;
                    stats.disconnectEvents.add(snapshot("visionConnected=" + visionConnected, ts, mode, matchTime, voltage, batteryLow, brownout,
                            gyroConnected, visionConnected, cam0Connected, cam1Connected, radioConnected,
                            joystickAxes, joystickButtons, requested, measured));
                }
            } else if (entry == cam0ConnectedEntry) {
                boolean newValue = record.getBoolean();
                if (newValue != cam0Connected) {
                    cam0Connected = newValue;
                    stats.disconnectEvents.add(snapshot("cam0Connected=" + cam0Connected, ts, mode, matchTime, voltage, batteryLow, brownout,
                            gyroConnected, visionConnected, cam0Connected, cam1Connected, radioConnected,
                            joystickAxes, joystickButtons, requested, measured));
                }
            } else if (entry == cam1ConnectedEntry) {
                boolean newValue = record.getBoolean();
                if (newValue != cam1Connected) {
                    cam1Connected = newValue;
                    stats.disconnectEvents.add(snapshot("cam1Connected=" + cam1Connected, ts, mode, matchTime, voltage, batteryLow, brownout,
                            gyroConnected, visionConnected, cam0Connected, cam1Connected, radioConnected,
                            joystickAxes, joystickButtons, requested, measured));
                }
            } else if (entry == radioConnectedEntry) {
                boolean newValue = readBooleanLenient(record, entries.get(entry).name);
                if (radioConnected == null || newValue != radioConnected.booleanValue()) {
                    radioConnected = newValue;
                    stats.disconnectEvents.add(snapshot("radioConnected=" + radioConnected, ts, mode, matchTime, voltage, batteryLow, brownout,
                            gyroConnected, visionConnected, cam0Connected, cam1Connected, radioConnected,
                            joystickAxes, joystickButtons, requested, measured));
                }
            } else if (handleModuleConnection(entry, record, entries, moduleDriveConnectedEntries, moduleDriveConnected,
                    "driveConnected", stats, ts, mode, matchTime, voltage, batteryLow, brownout,
                    gyroConnected, visionConnected, cam0Connected, cam1Connected, radioConnected,
                    joystickAxes, joystickButtons, requested, measured)) {
                // handled
            } else if (handleModuleConnection(entry, record, entries, moduleTurnConnectedEntries, moduleTurnConnected,
                    "turnConnected", stats, ts, mode, matchTime, voltage, batteryLow, brownout,
                    gyroConnected, visionConnected, cam0Connected, cam1Connected, radioConnected,
                    joystickAxes, joystickButtons, requested, measured)) {
                // handled
            } else if (handleModuleConnection(entry, record, entries, moduleEncoderConnectedEntries, moduleEncoderConnected,
                    "encoderConnected", stats, ts, mode, matchTime, voltage, batteryLow, brownout,
                    gyroConnected, visionConnected, cam0Connected, cam1Connected, radioConnected,
                    joystickAxes, joystickButtons, requested, measured)) {
                // handled
            } else if (entry == joystickAxesEntry) {
                joystickAxes = readDoubleArrayLenient(record);
                if (enabled && joystickAxes != null && joystickAxes.length >= 2) {
                    double mag = 0.0;
                    for (double v : joystickAxes) {
                        mag = Math.max(mag, Math.abs(v));
                    }
                    if (mag > 0.25 && requested != null && requested.linearMag() < 0.15 && stats.requestDropSnapshots.size() < 40) {
                        stats.requestDropSnapshots.add(snapshot("joystickActiveButRequestedLow", ts, mode, matchTime, voltage, batteryLow, brownout,
                                gyroConnected, visionConnected, cam0Connected, cam1Connected, radioConnected,
                                joystickAxes, joystickButtons, requested, measured));
                    }
                }
            } else if (entry == joystickButtonsEntry) {
                joystickButtons = readLongLenient(record);
            } else if (entry == joystickPovsEntry) {
                joystickPovs = readLongArrayLenient(record);
            } else if (entry == reqSpeedsEntry) {
                requested = readChassisSample(record, entries.get(entry).type);
                if (enabled && requested != null && measured != null
                        && requested.linearMag() > 1.0
                        && measured.linearMag() < requested.linearMag() * 0.35
                        && stats.responseMismatchSnapshots.size() < 80) {
                    stats.responseMismatchSnapshots.add(snapshot("requestedHighMeasuredLow", ts, mode, matchTime, voltage, batteryLow, brownout,
                            gyroConnected, visionConnected, cam0Connected, cam1Connected, radioConnected,
                            joystickAxes, joystickButtons, requested, measured));
                }
            } else if (entry == measSpeedsEntry) {
                measured = readChassisSample(record, entries.get(entry).type);
                if (enabled && requested != null && measured != null
                        && requested.linearMag() > 1.0
                        && measured.linearMag() < requested.linearMag() * 0.35
                        && stats.responseMismatchSnapshots.size() < 80) {
                    stats.responseMismatchSnapshots.add(snapshot("requestedHighMeasuredLow", ts, mode, matchTime, voltage, batteryLow, brownout,
                            gyroConnected, visionConnected, cam0Connected, cam1Connected, radioConnected,
                            joystickAxes, joystickButtons, requested, measured));
                }
            }
        }

        return stats.format(wpilog);
    }

    private static String snapshot(String label, long ts, String mode, double matchTime, double voltage,
            boolean batteryLow, boolean brownout, boolean gyroConnected, boolean visionConnected,
            boolean cam0Connected, boolean cam1Connected, Boolean radioConnected, double[] axes,
            long buttons, ChassisSample requested, ChassisSample measured) {
        return String.format(Locale.US,
                "%s t=%.3f mode=%s matchTime=%.1f V=%.2f batteryLow=%s brownout=%s gyro=%s vision=%s cam0=%s cam1=%s radio=%s axes=%s buttons=0x%s req=%s meas=%s",
                label,
                ts / 1_000_000.0,
                mode,
                matchTime,
                voltage,
                batteryLow,
                brownout,
                gyroConnected,
                visionConnected,
                cam0Connected,
                cam1Connected,
                radioConnected == null ? "<missing>" : radioConnected.toString(),
                formatAxes(axes),
                Long.toHexString(buttons),
                requested == null ? "<none>" : requested.toString(),
                measured == null ? "<none>" : measured.toString());
    }

    private static String formatAxes(double[] axes) {
        if (axes == null) {
            return "<none>";
        }
        StringBuilder out = new StringBuilder("[");
        for (int i = 0; i < axes.length; i++) {
            if (i > 0) {
                out.append(',');
            }
            out.append(String.format(Locale.US, "%.2f", axes[i]));
        }
        return out.append(']').toString();
    }

    private static boolean handleModuleConnection(
            int entry,
            DataLogRecord record,
            Map<Integer, EntryInfo> entries,
            int[] targetEntries,
            boolean[] states,
            String label,
            Stats stats,
            long ts,
            String mode,
            double matchTime,
            double voltage,
            boolean batteryLow,
            boolean brownout,
            boolean gyroConnected,
            boolean visionConnected,
            boolean cam0Connected,
            boolean cam1Connected,
            Boolean radioConnected,
            double[] joystickAxes,
            long joystickButtons,
            ChassisSample requested,
            ChassisSample measured) {
        for (int i = 0; i < targetEntries.length; i++) {
            if (entry != targetEntries[i]) {
                continue;
            }
            boolean newValue = readBooleanLenient(record, entries.get(entry).name);
            if (newValue != states[i]) {
                states[i] = newValue;
                stats.disconnectEvents.add(snapshot(
                        "module" + i + " " + label + "=" + newValue,
                        ts,
                        mode,
                        matchTime,
                        voltage,
                        batteryLow,
                        brownout,
                        gyroConnected,
                        visionConnected,
                        cam0Connected,
                        cam1Connected,
                        radioConnected,
                        joystickAxes,
                        joystickButtons,
                        requested,
                        measured));
            }
            return true;
        }
        return false;
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
                while (end < text.length() && (Character.isDigit(text.charAt(end)) || text.charAt(end) == '.' || text.charAt(end) == 'E' || text.charAt(end) == 'e' || text.charAt(end) == '-')) {
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

    private static double[] readDoubleArrayLenient(DataLogRecord record) {
        try {
            return record.getDoubleArray();
        } catch (Exception ignored) {
        }
        return null;
    }

    private static long[] readLongArrayLenient(DataLogRecord record) {
        try {
            return record.getIntegerArray();
        } catch (Exception ignored) {
        }
        return null;
    }

    private static long readLongLenient(DataLogRecord record) {
        try {
            return record.getInteger();
        } catch (Exception ignored) {
        }
        try {
            return (long) record.getDouble();
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

    private record EntryInfo(String name, String type) {}

    private record ChassisSample(double vx, double vy, double omega) {
        private double linearMag() {
            return Math.hypot(vx, vy);
        }

        @Override
        public String toString() {
            return String.format(Locale.US, "(vx=%.2f,vy=%.2f,om=%.2f)", vx, vy, omega);
        }
    }

    private static final class Stats {
        private final Map<String, EntryInfo> entries = new HashMap<>();
        private final List<String> disconnectEvents = new ArrayList<>();
        private final List<String> events = new ArrayList<>();
        private final List<String> lowVoltageSnapshots = new ArrayList<>();
        private final List<String> requestDropSnapshots = new ArrayList<>();
        private final List<String> responseMismatchSnapshots = new ArrayList<>();

        private String format(Path wpilog) {
            StringBuilder out = new StringBuilder();
            out.append("Stutter log summary\n");
            out.append("wpilog=").append(wpilog.toAbsolutePath()).append('\n');
            out.append("entries\n");
            for (var entry : entries.entrySet()) {
                out.append("  ").append(entry.getKey()).append(" = ");
                if (entry.getValue() == null) {
                    out.append("<missing>");
                } else {
                    out.append(entry.getValue().type).append(" | ").append(entry.getValue().name);
                }
                out.append('\n');
            }
            out.append("disconnect events\n");
            appendLines(out, disconnectEvents);
            out.append("state events\n");
            appendLines(out, events);
            out.append("low voltage snapshots\n");
            appendLines(out, lowVoltageSnapshots);
            out.append("joystick active but requested low\n");
            appendLines(out, requestDropSnapshots);
            out.append("requested high but measured low\n");
            appendLines(out, responseMismatchSnapshots);
            return out.toString();
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
    }
}
