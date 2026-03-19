package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.util.struct.StructBuffer;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import org.junit.jupiter.api.Test;

class AutoFeedShynessLogAnalyzerTest {
    @Test
    void analyze() throws Exception {
        Path logPath = Path.of("logs/akit_26-03-18_23-28-36.wpilog").toAbsolutePath();
        var recs = recordsUntilFailure(new DataLogReader(logPath.toString()));
        int poseEntry=-1, measEntry=-1, reqEntry=-1, aimReadyEntry=-1, shooterAtEntry=-1, gateEntry=-1, blockEntry=-1, stateEntry=-1, aimErrEntry=-1, eventEntry=-1, autoFeedEntry=-1, manualEntry=-1;
        for (var record: recs) {
            if (!record.isStart()) continue;
            try {
                var s=record.getStartData();
                switch (s.name) {
                    case "/RealOutputs/Odometry/Robot" -> poseEntry=s.entry;
                    case "/RealOutputs/SwerveChassisSpeeds/Measured" -> measEntry=s.entry;
                    case "/RealOutputs/SwerveChassisSpeeds/Requested" -> reqEntry=s.entry;
                    case "/RealOutputs/Shooting/AimReady" -> aimReadyEntry=s.entry;
                    case "/RealOutputs/Shooting/ShooterAtSetpoint" -> shooterAtEntry=s.entry;
                    case "/RealOutputs/Shooting/GateOpen" -> gateEntry=s.entry;
                    case "/RealOutputs/Shooting/BlockReason" -> blockEntry=s.entry;
                    case "/RealOutputs/Shooting/State" -> stateEntry=s.entry;
                    case "/RealOutputs/Shooting/AimErrorDeg" -> aimErrEntry=s.entry;
                    case "/RealOutputs/Commands/lastEvent" -> eventEntry=s.entry;
                    case "/RealOutputs/Shooting/AutomaticFeedEnabled" -> autoFeedEntry=s.entry;
                    case "/RealOutputs/Shooting/ManualFeedOverride" -> manualEntry=s.entry;
                    default -> {}
                }
            } catch (IllegalArgumentException ignored) {}
        }
        var poseBuf=StructBuffer.create(Pose2d.struct);
        var speedBuf=StructBuffer.create(ChassisSpeeds.struct);
        Pose2d pose=new Pose2d(); ChassisSpeeds meas=new ChassisSpeeds(); ChassisSpeeds req=new ChassisSpeeds();
        boolean aimReady=false, shooterAt=false, gate=false, autoFeed=false, manual=false; String block="", state="", event=""; double aimErr=Double.NaN;
        record Snap(double t, Pose2d pose, ChassisSpeeds meas, ChassisSpeeds req, boolean aimReady, boolean shooterAt, boolean gate, boolean autoFeed, boolean manual, String block, String state, double aimErr, String event){}
        List<Snap> snaps=new ArrayList<>();
        for (var r: recs) {
            if (r.isStart()||r.isControl()) continue;
            int e=r.getEntry();
            if (e==poseEntry) pose=poseBuf.read(r.getRaw());
            else if (e==measEntry) meas=speedBuf.read(r.getRaw());
            else if (e==reqEntry) req=speedBuf.read(r.getRaw());
            else if (e==aimReadyEntry) aimReady=r.getBoolean();
            else if (e==shooterAtEntry) shooterAt=r.getBoolean();
            else if (e==gateEntry) gate=r.getBoolean();
            else if (e==autoFeedEntry) autoFeed=r.getBoolean();
            else if (e==manualEntry) manual=r.getBoolean();
            else if (e==blockEntry) block=r.getString();
            else if (e==stateEntry) state=r.getString();
            else if (e==aimErrEntry) aimErr=r.getDouble();
            else if (e==eventEntry) event=r.getString();
            else continue;
            snaps.add(new Snap(r.getTimestamp()/1_000_000.0, pose, meas, req, aimReady, shooterAt, gate, autoFeed, manual, block, state, aimErr, event));
        }
        double[] interestingStarts={17.028,22.115,30.586,38.848,45.534,50.921};
        for (double start: interestingStarts) {
            System.out.println("\n=== around auto-feed start "+start+" ===");
            double last=-1e9;
            for (var s: snaps) {
                if (s.t<start-0.2||s.t>start+1.4) continue;
                if (s.t-last<0.08) continue;
                System.out.printf("t=%.3f reqLin=%.2f measLin=%.2f aimReady=%s shooterAt=%s gate=%s autoFeed=%s manual=%s aimErr=%.2f block=%s state=%s pose=(%.2f,%.2f,%.1f) event=%s%n",
                    s.t,
                    Math.hypot(s.req.vxMetersPerSecond,s.req.vyMetersPerSecond),
                    Math.hypot(s.meas.vxMetersPerSecond,s.meas.vyMetersPerSecond),
                    s.aimReady,s.shooterAt,s.gate,s.autoFeed,s.manual,s.aimErr,s.block,s.state,
                    s.pose.getX(),s.pose.getY(),s.pose.getRotation().getDegrees(),s.event);
                last=s.t;
            }
        }
    }
    private static List<DataLogRecord> recordsUntilFailure(DataLogReader reader) {
        List<DataLogRecord> records=new ArrayList<>();
        Iterator<DataLogRecord> it=reader.iterator();
        while (true) {
            try {
                if (!it.hasNext()) break;
                records.add(it.next());
            } catch (IllegalArgumentException ex) { break; }
        }
        return records;
    }
}
