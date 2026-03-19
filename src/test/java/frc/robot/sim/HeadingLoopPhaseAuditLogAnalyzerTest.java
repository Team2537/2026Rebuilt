package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.util.struct.StructBuffer;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import org.junit.jupiter.api.Test;

class HeadingLoopPhaseAuditLogAnalyzerTest {
    @Test
    void auditShotYawPhaseAndContributionBreakdown() throws Exception {
        Path logPath = Path.of("logs/akit_26-03-18_23-28-36.wpilog").toAbsolutePath();
        List<DataLogRecord> records = recordsUntilFailure(new DataLogReader(logPath.toString()));
        int poseEntry=-1, autoFeedEntry=-1, manualEntry=-1, aimErrEntry=-1, desiredRateEntry=-1, measuredOmegaEntry=-1,
                omegaCmdEntry=-1, kPEntry=-1, kDEntry=-1, stateEntry=-1;
        for (var record : records) {
            if (!record.isStart()) continue;
            try {
                var s = record.getStartData();
                switch (s.name) {
                    case "/RealOutputs/Odometry/Robot" -> poseEntry=s.entry;
                    case "/RealOutputs/Shooting/AutomaticFeedEnabled" -> autoFeedEntry=s.entry;
                    case "/RealOutputs/Shooting/ManualFeedOverride" -> manualEntry=s.entry;
                    case "/RealOutputs/Shooting/AimErrorDeg" -> aimErrEntry=s.entry;
                    case "/RealOutputs/ShotYaw/DesiredHeadingRateRadPerSec" -> desiredRateEntry=s.entry;
                    case "/RealOutputs/ShotYaw/MeasuredOmegaRadPerSec" -> measuredOmegaEntry=s.entry;
                    case "/RealOutputs/ShotYaw/OmegaCommandRadPerSec" -> omegaCmdEntry=s.entry;
                    case "/RealOutputs/ShotYaw/kP" -> kPEntry=s.entry;
                    case "/RealOutputs/ShotYaw/kD" -> kDEntry=s.entry;
                    case "/RealOutputs/Shooting/State" -> stateEntry=s.entry;
                    default -> {}
                }
            } catch (IllegalArgumentException ignored) {}
        }

        var poseBuf = StructBuffer.create(Pose2d.struct);
        Pose2d pose = new Pose2d();
        boolean autoFeed=false, manual=false;
        double aimErrDeg=Double.NaN, desiredRate=Double.NaN, measuredOmega=Double.NaN, omegaCmd=Double.NaN, kP=Double.NaN, kD=Double.NaN;
        String state="";
        record Snap(double t, Pose2d pose, boolean autoFeed, boolean manual, double aimErrDeg, double desiredRate, double measuredOmega,
                    double omegaCmd, double kP, double kD, String state) {}
        List<Snap> snaps = new ArrayList<>();
        for (var r : records) {
            if (r.isStart() || r.isControl()) continue;
            int e = r.getEntry();
            if (e==poseEntry) pose = poseBuf.read(r.getRaw());
            else if (e==autoFeedEntry) autoFeed = r.getBoolean();
            else if (e==manualEntry) manual = r.getBoolean();
            else if (e==aimErrEntry) aimErrDeg = r.getDouble();
            else if (e==desiredRateEntry) desiredRate = r.getDouble();
            else if (e==measuredOmegaEntry) measuredOmega = r.getDouble();
            else if (e==omegaCmdEntry) omegaCmd = r.getDouble();
            else if (e==kPEntry) kP = r.getDouble();
            else if (e==kDEntry) kD = r.getDouble();
            else if (e==stateEntry) state = r.getString();
            else continue;
            snaps.add(new Snap(r.getTimestamp()/1_000_000.0, pose, autoFeed, manual, aimErrDeg, desiredRate, measuredOmega, omegaCmd, kP, kD, state));
        }

        double[][] windows = {{22.0,23.3},{30.3,31.7},{38.6,40.2},{50.7,51.8}};
        for (double[] w : windows) {
            System.out.printf("\n=== heading phase %.3f-%.3f ===%n", w[0], w[1]);
            int wrongWayCount = 0;
            int meaningfulCount = 0;
            double last=-1e9;
            for (var s : snaps) {
                if (s.t < w[0] || s.t > w[1]) continue;
                if (!s.autoFeed || s.manual) continue;
                if (!Double.isFinite(s.kP) || !Double.isFinite(s.kD)) continue;
                double errorRad = Math.toRadians(s.aimErrDeg);
                double p = s.kP * errorRad;
                double d = s.kD * (s.desiredRate - s.measuredOmega);
                double ff = s.desiredRate;
                double unclamped = ff + p + d;
                boolean meaningfulError = Math.abs(s.aimErrDeg) >= 5.0;
                boolean wrongWay = meaningfulError && Math.signum(errorRad) != 0.0 && Math.signum(s.omegaCmd) != 0.0
                        && Math.signum(errorRad) != Math.signum(s.omegaCmd);
                if (meaningfulError) meaningfulCount++;
                if (wrongWay) wrongWayCount++;
                if (s.t - last < 0.08) continue;
                System.out.printf(
                        "t=%.3f pos=(%.2f,%.2f,%.1f) err=%.2fdeg ff=%.2f p=%.2f d=%.2f sum=%.2f cmd=%.2f measW=%.2f wrongWay=%s state=%s%n",
                        s.t,
                        s.pose.getX(), s.pose.getY(), s.pose.getRotation().getDegrees(),
                        s.aimErrDeg,
                        ff,
                        p,
                        d,
                        unclamped,
                        s.omegaCmd,
                        s.measuredOmega,
                        wrongWay,
                        s.state);
                last = s.t;
            }
            System.out.printf("meaningfulErrSamples=%d wrongWayCmdSamples=%d\n", meaningfulCount, wrongWayCount);
        }
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
