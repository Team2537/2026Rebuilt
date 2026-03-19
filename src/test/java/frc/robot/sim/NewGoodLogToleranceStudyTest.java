package frc.robot.sim;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import org.junit.jupiter.api.Test;

class NewGoodLogToleranceStudyTest {
    @Test
    void studyMovingAimToleranceMargin() throws Exception {
        Path logPath = Path.of("logs/akit_26-03-19_00-08-50.wpilog").toAbsolutePath();
        List<DataLogRecord> records = recordsUntilFailure(new DataLogReader(logPath.toString()));
        int aimErrEntry=-1, autoFeedEntry=-1, manualEntry=-1, gateEntry=-1, blockEntry=-1, tolEntry=-1, releaseTolEntry=-1, measEntry=-1, stateEntry=-1;
        for (var record : records) {
            if (!record.isStart()) continue;
            try {
                var s = record.getStartData();
                switch (s.name) {
                    case "/RealOutputs/Shooting/AimErrorDeg" -> aimErrEntry = s.entry;
                    case "/RealOutputs/Shooting/AutomaticFeedEnabled" -> autoFeedEntry = s.entry;
                    case "/RealOutputs/Shooting/ManualFeedOverride" -> manualEntry = s.entry;
                    case "/RealOutputs/Shooting/GateOpen" -> gateEntry = s.entry;
                    case "/RealOutputs/Shooting/BlockReason" -> blockEntry = s.entry;
                    case "/RealOutputs/Shooting/ActiveAimToleranceRad" -> tolEntry = s.entry;
                    case "/RealOutputs/Shooting/ActiveAimReleaseToleranceRad" -> releaseTolEntry = s.entry;
                    case "/RealOutputs/SwerveChassisSpeeds/Measured" -> measEntry = s.entry;
                    case "/RealOutputs/Shooting/State" -> stateEntry = s.entry;
                    default -> {}
                }
            } catch (IllegalArgumentException ignored) {}
        }
        boolean autoFeed=false, manual=false, gate=false;
        String block="", state="";
        double aimErr=Double.NaN, tolDeg=Double.NaN, relTolDeg=Double.NaN, speed=0.0;
        var speedsBuf = edu.wpi.first.util.struct.StructBuffer.create(edu.wpi.first.math.kinematics.ChassisSpeeds.struct);
        List<Double> gateOpenMoving = new ArrayList<>();
        List<Double> aimBlockedMoving = new ArrayList<>();
        List<Double> allMoving = new ArrayList<>();
        int movingSamples=0, movingGateSamples=0, movingAimBlockedSamples=0;
        for (var r : records) {
            if (r.isStart() || r.isControl()) continue;
            int e = r.getEntry();
            if (e==aimErrEntry) aimErr = r.getDouble();
            else if (e==autoFeedEntry) autoFeed = r.getBoolean();
            else if (e==manualEntry) manual = r.getBoolean();
            else if (e==gateEntry) gate = r.getBoolean();
            else if (e==blockEntry) block = r.getString();
            else if (e==tolEntry) tolDeg = Math.toDegrees(r.getDouble());
            else if (e==releaseTolEntry) relTolDeg = Math.toDegrees(r.getDouble());
            else if (e==measEntry) {
                var s = speedsBuf.read(r.getRaw());
                speed = Math.hypot(s.vxMetersPerSecond, s.vyMetersPerSecond);
            } else if (e==stateEntry) state = r.getString();
            else continue;

            boolean moving = speed >= 0.25;
            if (!(autoFeed && !manual && moving && Double.isFinite(aimErr))) continue;
            double absErr = Math.abs(aimErr);
            movingSamples++;
            allMoving.add(absErr);
            if (gate) {
                movingGateSamples++;
                gateOpenMoving.add(absErr);
            }
            if (!gate && block.contains("AimNotReady")) {
                movingAimBlockedSamples++;
                aimBlockedMoving.add(absErr);
            }
        }
        System.out.printf("moving samples=%d gateOpen=%d aimBlocked=%d tol=%.2f relTol=%.2f%n",
                movingSamples, movingGateSamples, movingAimBlockedSamples, tolDeg, relTolDeg);
        printStats("allMovingAbsErr", allMoving);
        printStats("gateOpenMovingAbsErr", gateOpenMoving);
        printStats("aimBlockedMovingAbsErr", aimBlockedMoving);
        System.out.printf("gate-open >4deg count=%d / %d%n", countAbove(gateOpenMoving, 4.0), gateOpenMoving.size());
        System.out.printf("gate-open >5deg count=%d / %d%n", countAbove(gateOpenMoving, 5.0), gateOpenMoving.size());
        System.out.printf("aimBlocked <=4deg count=%d / %d%n", countAtOrBelow(aimBlockedMoving, 4.0), aimBlockedMoving.size());
        System.out.printf("aimBlocked <=5deg count=%d / %d%n", countAtOrBelow(aimBlockedMoving, 5.0), aimBlockedMoving.size());
    }

    private static void printStats(String label, List<Double> values) {
        if (values.isEmpty()) {
            System.out.println(label + ": empty");
            return;
        }
        List<Double> sorted = new ArrayList<>(values);
        Collections.sort(sorted);
        System.out.printf("%s: min=%.2f p50=%.2f p75=%.2f p90=%.2f p95=%.2f max=%.2f%n",
                label,
                sorted.get(0),
                percentile(sorted, 0.50),
                percentile(sorted, 0.75),
                percentile(sorted, 0.90),
                percentile(sorted, 0.95),
                sorted.get(sorted.size()-1));
    }

    private static double percentile(List<Double> sorted, double p) {
        int idx = (int)Math.round((sorted.size()-1)*p);
        return sorted.get(Math.max(0, Math.min(sorted.size()-1, idx)));
    }

    private static long countAbove(List<Double> values, double threshold) {
        return values.stream().filter(v -> v > threshold).count();
    }

    private static long countAtOrBelow(List<Double> values, double threshold) {
        return values.stream().filter(v -> v <= threshold).count();
    }

    private static List<DataLogRecord> recordsUntilFailure(DataLogReader reader) {
        List<DataLogRecord> records = new ArrayList<>();
        Iterator<DataLogRecord> it = reader.iterator();
        while (true) {
            try {
                if (!it.hasNext()) break;
                records.add(it.next());
            } catch (IllegalArgumentException ex) {
                break;
            }
        }
        return records;
    }
}
