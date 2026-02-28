package frc.robot.autos;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import org.junit.jupiter.api.Test;

class ShooterGateLogAnalyzerTest {
    @Test
    void analyzeShooterGateTimelineFromWpiLog() throws IOException {
        String wpilogPath = System.getProperty("gateDiag.wpilog");
        if (wpilogPath == null || wpilogPath.isBlank()) {
            wpilogPath = System.getenv("GATE_DIAG_WPILOG");
        }
        assertTrue(wpilogPath != null && !wpilogPath.isBlank(),
                "Missing gate path. Set -DgateDiag.wpilog=<path> or GATE_DIAG_WPILOG=<path>.");

        Path wpilog = Path.of(wpilogPath).toAbsolutePath();
        assertTrue(Files.exists(wpilog), "WPILOG does not exist: " + wpilog);

        ShooterGateSummary summary = extractShooterGateSummary(wpilog);
        String report = formatShooterSummary(wpilog, summary);

        String outPath = System.getProperty("gateDiag.out");
        if (outPath == null || outPath.isBlank()) {
            outPath = System.getenv("GATE_DIAG_OUT");
        }
        if (outPath != null && !outPath.isBlank()) {
            Path out = Path.of(outPath).toAbsolutePath();
            Files.createDirectories(out.getParent());
            Files.writeString(out, report);
            System.out.println("Shooter gate summary written: " + out);
        }
        System.out.println(report);
    }

    private static ShooterGateSummary extractShooterGateSummary(Path wpilog) throws IOException {
        DataLogReader reader = new DataLogReader(wpilog.toString());
        if (!reader.isValid()) {
            throw new IllegalStateException("Invalid WPILOG file: " + wpilog);
        }

        int distanceValidEntry = -1;
        int gateOpenEntry = -1;
        int blockReasonEntry = -1;
        int shooterAtSetpointEntry = -1;
        int aimReadyEntry = -1;
        int leftVelocityErrorEntry = -1;
        int rightVelocityErrorEntry = -1;
        int hoodAngleErrorEntry = -1;
        int shooterTargetLeftEntry = -1;
        int shooterTargetRightEntry = -1;
        int kickerTorqueEntry = -1;
        String distanceValidKey = "";
        String gateOpenKey = "";
        String blockReasonKey = "";
        String shooterAtSetpointKey = "";
        String aimReadyKey = "";
        String leftVelocityErrorKey = "";
        String rightVelocityErrorKey = "";
        String hoodAngleErrorKey = "";
        String shooterTargetLeftKey = "";
        String shooterTargetRightKey = "";
        String kickerTorqueKey = "";

        for (DataLogRecord record : reader) {
            if (!record.isStart()) {
                continue;
            }
            var start = record.getStartData();
            String name = start.name;
            if (name.contains("Shooting/DistanceValid")) {
                distanceValidEntry = start.entry;
                distanceValidKey = name;
            } else if (name.contains("Shooting/GateOpen")) {
                gateOpenEntry = start.entry;
                gateOpenKey = name;
            } else if (name.contains("Shooting/BlockReason")) {
                blockReasonEntry = start.entry;
                blockReasonKey = name;
            } else if (name.contains("Shooting/ShooterAtSetpoint")) {
                shooterAtSetpointEntry = start.entry;
                shooterAtSetpointKey = name;
            } else if (name.contains("Shooting/AimReady")) {
                aimReadyEntry = start.entry;
                aimReadyKey = name;
            } else if (name.contains("Shooter/Readiness/LeftVelocityErrorRpm")) {
                leftVelocityErrorEntry = start.entry;
                leftVelocityErrorKey = name;
            } else if (name.contains("Shooter/Readiness/RightVelocityErrorRpm")) {
                rightVelocityErrorEntry = start.entry;
                rightVelocityErrorKey = name;
            } else if (name.contains("Shooter/Readiness/HoodAngleErrorDeg")) {
                hoodAngleErrorEntry = start.entry;
                hoodAngleErrorKey = name;
            } else if (name.contains("Shooter/TargetLeftRpm")) {
                shooterTargetLeftEntry = start.entry;
                shooterTargetLeftKey = name;
            } else if (name.contains("Shooter/TargetRightRpm")) {
                shooterTargetRightEntry = start.entry;
                shooterTargetRightKey = name;
            } else if (name.contains("Shooter/KickerTorqueAmps")) {
                kickerTorqueEntry = start.entry;
                kickerTorqueKey = name;
            }
        }

        boolean distanceValid = false;
        boolean gateOpen = false;
        String blockReason = "";
        boolean shooterAtSetpoint = false;
        boolean aimReady = false;
        double leftVelocityErrorRpm = Double.NaN;
        double rightVelocityErrorRpm = Double.NaN;
        double hoodAngleErrorDeg = Double.NaN;
        double shooterTargetLeftRpm = Double.NaN;
        double shooterTargetRightRpm = Double.NaN;
        double kickerTorqueAmps = 0.0;

        long firstDistanceValidUs = Long.MIN_VALUE;
        long firstGateOpenUs = Long.MIN_VALUE;
        long firstKickerActiveUs = Long.MIN_VALUE;
        Map<String, Integer> preOpenBlockReasonCounts = new LinkedHashMap<>();
        List<String> preOpenTransitions = new ArrayList<>();

        for (DataLogRecord record : reader) {
            if (record.isStart() || record.isControl()) {
                continue;
            }

            int entry = record.getEntry();
            long timestampUs = record.getTimestamp();
            boolean relevantUpdate = false;

            if (entry == distanceValidEntry) {
                distanceValid = record.getBoolean();
                relevantUpdate = true;
                if (distanceValid && firstDistanceValidUs == Long.MIN_VALUE) {
                    firstDistanceValidUs = timestampUs;
                }
            } else if (entry == gateOpenEntry) {
                gateOpen = record.getBoolean();
                relevantUpdate = true;
                if (gateOpen
                        && firstGateOpenUs == Long.MIN_VALUE
                        && firstDistanceValidUs != Long.MIN_VALUE) {
                    firstGateOpenUs = timestampUs;
                }
            } else if (entry == blockReasonEntry) {
                blockReason = record.getString();
                relevantUpdate = true;
                if (firstDistanceValidUs != Long.MIN_VALUE && firstGateOpenUs == Long.MIN_VALUE) {
                    if (blockReason != null && !blockReason.isBlank()) {
                        preOpenBlockReasonCounts.merge(blockReason, 1, Integer::sum);
                    }
                }
            } else if (entry == shooterAtSetpointEntry) {
                shooterAtSetpoint = record.getBoolean();
                relevantUpdate = true;
            } else if (entry == aimReadyEntry) {
                aimReady = record.getBoolean();
                relevantUpdate = true;
            } else if (entry == leftVelocityErrorEntry) {
                leftVelocityErrorRpm = record.getDouble();
                relevantUpdate = true;
            } else if (entry == rightVelocityErrorEntry) {
                rightVelocityErrorRpm = record.getDouble();
                relevantUpdate = true;
            } else if (entry == hoodAngleErrorEntry) {
                hoodAngleErrorDeg = record.getDouble();
                relevantUpdate = true;
            } else if (entry == shooterTargetLeftEntry) {
                shooterTargetLeftRpm = record.getDouble();
                relevantUpdate = true;
            } else if (entry == shooterTargetRightEntry) {
                shooterTargetRightRpm = record.getDouble();
                relevantUpdate = true;
            } else if (entry == kickerTorqueEntry) {
                kickerTorqueAmps = record.getDouble();
                relevantUpdate = true;
                if (Math.abs(kickerTorqueAmps) > 1e-6 && firstKickerActiveUs == Long.MIN_VALUE) {
                    firstKickerActiveUs = timestampUs;
                }
            }

            if (!relevantUpdate) {
                continue;
            }

            if (firstDistanceValidUs != Long.MIN_VALUE && firstGateOpenUs == Long.MIN_VALUE) {
                preOpenTransitions.add(String.format(
                        Locale.US,
                        "  t=%.3f gateOpen=%s blockReason=%s shooterAtSetpoint=%s aimReady=%s "
                                + "leftErr=%.1f rightErr=%.1f hoodErrDeg=%.2f targetL=%.1f targetR=%.1f",
                        timestampUs / 1_000_000.0,
                        gateOpen,
                        blockReason == null || blockReason.isBlank() ? "<none>" : blockReason,
                        shooterAtSetpoint,
                        aimReady,
                        leftVelocityErrorRpm,
                        rightVelocityErrorRpm,
                        hoodAngleErrorDeg,
                        shooterTargetLeftRpm,
                        shooterTargetRightRpm));
            }
        }

        double distanceToOpenSec = Double.NaN;
        if (firstDistanceValidUs != Long.MIN_VALUE && firstGateOpenUs != Long.MIN_VALUE) {
            distanceToOpenSec = (firstGateOpenUs - firstDistanceValidUs) / 1_000_000.0;
        }
        double distanceToKickerSec = Double.NaN;
        if (firstDistanceValidUs != Long.MIN_VALUE && firstKickerActiveUs != Long.MIN_VALUE) {
            distanceToKickerSec = (firstKickerActiveUs - firstDistanceValidUs) / 1_000_000.0;
        }

        return new ShooterGateSummary(
                firstDistanceValidUs,
                firstGateOpenUs,
                firstKickerActiveUs,
                distanceToOpenSec,
                distanceToKickerSec,
                preOpenBlockReasonCounts,
                preOpenTransitions,
                distanceValidKey,
                gateOpenKey,
                blockReasonKey,
                shooterAtSetpointKey,
                aimReadyKey,
                leftVelocityErrorKey,
                rightVelocityErrorKey,
                hoodAngleErrorKey,
                shooterTargetLeftKey,
                shooterTargetRightKey,
                kickerTorqueKey);
    }

    private static String formatShooterSummary(Path wpilog, ShooterGateSummary summary) {
        StringBuilder blockCounts = new StringBuilder();
        if (summary.preOpenBlockReasonCounts().isEmpty()) {
            blockCounts.append("  (none)\n");
        } else {
            for (var entry : summary.preOpenBlockReasonCounts().entrySet()) {
                blockCounts.append(
                        String.format(Locale.US, "  %s: %d%n", entry.getKey(), entry.getValue()));
            }
        }

        StringBuilder transitions = new StringBuilder();
        if (summary.preOpenTransitions().isEmpty()) {
            transitions.append("  (none)\n");
        } else {
            for (String line : summary.preOpenTransitions()) {
                transitions.append(line).append(System.lineSeparator());
            }
        }

        return String.format(
                Locale.US,
                "wpilog=%s%n"
                        + "mapped_keys.distance_valid=%s%n"
                        + "mapped_keys.gate_open=%s%n"
                        + "mapped_keys.block_reason=%s%n"
                        + "mapped_keys.shooter_at_setpoint=%s%n"
                        + "mapped_keys.aim_ready=%s%n"
                        + "mapped_keys.left_velocity_error=%s%n"
                        + "mapped_keys.right_velocity_error=%s%n"
                        + "mapped_keys.hood_angle_error=%s%n"
                        + "mapped_keys.target_left=%s%n"
                        + "mapped_keys.target_right=%s%n"
                        + "mapped_keys.kicker_torque=%s%n"
                        + "first_distance_valid_us=%d%n"
                        + "first_gate_open_us=%d%n"
                        + "first_kicker_active_us=%d%n"
                        + "distance_to_gate_open_sec=%.3f%n"
                        + "distance_to_kicker_active_sec=%.3f%n"
                        + "%n"
                        + "pre_open_block_reason_counts=%n"
                        + "%s"
                        + "%n"
                        + "pre_open_transitions=%n"
                        + "%s",
                wpilog.toAbsolutePath(),
                summary.distanceValidKey(),
                summary.gateOpenKey(),
                summary.blockReasonKey(),
                summary.shooterAtSetpointKey(),
                summary.aimReadyKey(),
                summary.leftVelocityErrorKey(),
                summary.rightVelocityErrorKey(),
                summary.hoodAngleErrorKey(),
                summary.shooterTargetLeftKey(),
                summary.shooterTargetRightKey(),
                summary.kickerTorqueKey(),
                summary.firstDistanceValidUs(),
                summary.firstGateOpenUs(),
                summary.firstKickerActiveUs(),
                summary.distanceValidToGateOpenSec(),
                summary.distanceValidToKickerActiveSec(),
                blockCounts.toString(),
                transitions.toString());
    }

    private record ShooterGateSummary(
            long firstDistanceValidUs,
            long firstGateOpenUs,
            long firstKickerActiveUs,
            double distanceValidToGateOpenSec,
            double distanceValidToKickerActiveSec,
            Map<String, Integer> preOpenBlockReasonCounts,
            List<String> preOpenTransitions,
            String distanceValidKey,
            String gateOpenKey,
            String blockReasonKey,
            String shooterAtSetpointKey,
            String aimReadyKey,
            String leftVelocityErrorKey,
            String rightVelocityErrorKey,
            String hoodAngleErrorKey,
            String shooterTargetLeftKey,
            String shooterTargetRightKey,
            String kickerTorqueKey) {
    }
}
