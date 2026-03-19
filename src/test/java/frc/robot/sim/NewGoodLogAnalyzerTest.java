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

class NewGoodLogAnalyzerTest {
    private record Snap(double t, Pose2d pose, ChassisSpeeds meas, ChassisSpeeds req, boolean aimReady, boolean shooterAt, boolean gate,
                        boolean autoFeed, boolean manual, String block, String state, double aimErr, double desiredHeading,
                        double compHeading, double vCross, double distance, double tol, double releaseTol, String event) {}

    @Test
    void analyze() throws Exception {
        Path logPath = Path.of("logs/akit_26-03-19_00-08-50.wpilog").toAbsolutePath();
        List<DataLogRecord> records = recordsUntilFailure(new DataLogReader(logPath.toString()));
        int poseEntry=-1, measEntry=-1, reqEntry=-1, aimReadyEntry=-1, gateEntry=-1, blockEntry=-1, stateEntry=-1,
                aimErrEntry=-1, autoFeedEntry=-1, manualEntry=-1, desiredHeadingEntry=-1, compHeadingEntry=-1,
                vCrossEntry=-1, distanceEntry=-1, tolEntry=-1, releaseTolEntry=-1, shooterAtEntry=-1, eventEntry=-1;
        for (var record : records) {
            if (!record.isStart()) continue;
            try {
                var s = record.getStartData();
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
                    case "/RealOutputs/Shooting/AutomaticFeedEnabled" -> autoFeedEntry=s.entry;
                    case "/RealOutputs/Shooting/ManualFeedOverride" -> manualEntry=s.entry;
                    case "/RealOutputs/Shooting/DesiredRobotHeadingDeg" -> desiredHeadingEntry=s.entry;
                    case "/RealOutputs/Shooter/CompensatedHubHeadingDeg" -> compHeadingEntry=s.entry;
                    case "/RealOutputs/Shooter/VelocityPerpendicularHubMps" -> vCrossEntry=s.entry;
                    case "/RealOutputs/Shooting/DistanceMeters" -> distanceEntry=s.entry;
                    case "/RealOutputs/Shooting/ActiveAimToleranceRad" -> tolEntry=s.entry;
                    case "/RealOutputs/Shooting/ActiveAimReleaseToleranceRad" -> releaseTolEntry=s.entry;
                    case "/RealOutputs/Commands/lastEvent" -> eventEntry=s.entry;
                    default -> {}
                }
            } catch (IllegalArgumentException ignored) {}
        }
        var poseBuf = StructBuffer.create(Pose2d.struct);
        var speedBuf = StructBuffer.create(ChassisSpeeds.struct);
        Pose2d pose = new Pose2d();
        ChassisSpeeds meas = new ChassisSpeeds();
        ChassisSpeeds req = new ChassisSpeeds();
        boolean aimReady=false, shooterAt=false, gate=false, autoFeed=false, manual=false;
        String block="", state="", event="";
        double aimErr=Double.NaN, desiredHeading=Double.NaN, compHeading=Double.NaN, vCross=Double.NaN, distance=Double.NaN, tol=Double.NaN, releaseTol=Double.NaN;
        List<Snap> snaps = new ArrayList<>();
        for (var r : records) {
            if (r.isStart() || r.isControl()) continue;
            int e = r.getEntry();
            if (e==poseEntry) pose = poseBuf.read(r.getRaw());
            else if (e==measEntry) meas = speedBuf.read(r.getRaw());
            else if (e==reqEntry) req = speedBuf.read(r.getRaw());
            else if (e==aimReadyEntry) aimReady = r.getBoolean();
            else if (e==shooterAtEntry) shooterAt = r.getBoolean();
            else if (e==gateEntry) gate = r.getBoolean();
            else if (e==blockEntry) block = r.getString();
            else if (e==stateEntry) state = r.getString();
            else if (e==aimErrEntry) aimErr = r.getDouble();
            else if (e==autoFeedEntry) autoFeed = r.getBoolean();
            else if (e==manualEntry) manual = r.getBoolean();
            else if (e==desiredHeadingEntry) desiredHeading = r.getDouble();
            else if (e==compHeadingEntry) compHeading = r.getDouble();
            else if (e==vCrossEntry) vCross = r.getDouble();
            else if (e==distanceEntry) distance = r.getDouble();
            else if (e==tolEntry) tol = r.getDouble();
            else if (e==releaseTolEntry) releaseTol = r.getDouble();
            else if (e==eventEntry) event = r.getString();
            else continue;
            snaps.add(new Snap(r.getTimestamp()/1_000_000.0, pose, meas, req, aimReady, shooterAt, gate, autoFeed, manual, block, state,
                    aimErr, desiredHeading, compHeading, vCross, distance, tol, releaseTol, event));
        }

        boolean inAuto=false;
        double start=0;
        List<Snap> segment=new ArrayList<>();
        for (var s: snaps) {
            boolean active = s.autoFeed && !s.manual;
            if (active && !inAuto) {
                inAuto = true;
                start = s.t;
                segment = new ArrayList<>();
            }
            if (inAuto && active) segment.add(s);
            if (inAuto && !active) {
                summarize(start, segment);
                inAuto = false;
            }
        }
        if (inAuto) summarize(start, segment);
    }

    private static void summarize(double start, List<Snap> segment) {
        if (segment.isEmpty()) return;
        double end = segment.get(segment.size()-1).t;
        double maxErr=0, meanErr=0, maxSpeed=0, maxCross=0;
        int gateOpen=0;
        Snap firstGate = null;
        for (var s: segment) {
            maxErr = Math.max(maxErr, Math.abs(s.aimErr));
            meanErr += Math.abs(s.aimErr);
            maxSpeed = Math.max(maxSpeed, Math.hypot(s.meas.vxMetersPerSecond, s.meas.vyMetersPerSecond));
            maxCross = Math.max(maxCross, Math.abs(s.vCross));
            if (s.gate) {
                gateOpen++;
                if (firstGate == null) firstGate = s;
            }
        }
        meanErr /= segment.size();
        var first = segment.get(0);
        var last = segment.get(segment.size()-1);
        System.out.printf("\n=== autoFeed %.3f-%.3f dur=%.3fs samples=%d gateSamples=%d maxSpeed=%.2f maxCross=%.2f tol=%.2fdeg relTol=%.2fdeg ===%n",
                start, end, end-start, segment.size(), gateOpen, maxSpeed, maxCross,
                Math.toDegrees(first.tol), Math.toDegrees(first.releaseTol));
        System.out.printf("meanAbsAimErr=%.2f maxAbsAimErr=%.2f firstState=%s firstBlock=%s firstEvent=%s%n",
                meanErr, maxErr, first.state, first.block, first.event);
        if (firstGate != null) {
            System.out.printf("firstGate at %.3f aimErr=%.2f speed=%.2f cross=%.2f dist=%.2f pose=(%.2f,%.2f,%.1f) state=%s%n",
                    firstGate.t, firstGate.aimErr,
                    Math.hypot(firstGate.meas.vxMetersPerSecond, firstGate.meas.vyMetersPerSecond),
                    firstGate.vCross, firstGate.distance,
                    firstGate.pose.getX(), firstGate.pose.getY(), firstGate.pose.getRotation().getDegrees(), firstGate.state);
        }
        double lastPrint=-1e9;
        for (var s: segment) {
            if (s.t-lastPrint < 0.10) continue;
            System.out.printf("t=%.3f speed=%.2f cross=%.2f aimErr=%.2f aimReady=%s shooterAt=%s gate=%s block=%s state=%s dist=%.2f%n",
                    s.t,
                    Math.hypot(s.meas.vxMetersPerSecond, s.meas.vyMetersPerSecond),
                    s.vCross,
                    s.aimErr,
                    s.aimReady,
                    s.shooterAt,
                    s.gate,
                    s.block,
                    s.state,
                    s.distance);
            lastPrint = s.t;
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
