package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.util.struct.StructBuffer;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.FieldConstants;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import org.junit.jupiter.api.Test;

class AimLeadConsistencyLogAnalyzerTest {
    @Test
    void compareObservedLeadToVelocityBasedExpectation() throws Exception {
        Path logPath = Path.of("logs/akit_26-03-18_23-28-36.wpilog").toAbsolutePath();
        List<DataLogRecord> records = recordsUntilFailure(new DataLogReader(logPath.toString()));
        int poseEntry=-1, rawDistEntry=-1, compDistEntry=-1, timeEntry=-1, compHeadingEntry=-1,
                vTowEntry=-1, vCrossEntry=-1, autoFeedEntry=-1, manualEntry=-1, aimErrEntry=-1;
        for (var record : records) {
            if (!record.isStart()) continue;
            try {
                var s = record.getStartData();
                switch (s.name) {
                    case "/RealOutputs/Odometry/Robot" -> poseEntry=s.entry;
                    case "/RealOutputs/Shooter/RawHubDistanceMeters" -> rawDistEntry=s.entry;
                    case "/RealOutputs/Shooter/CompensatedHubDistanceMeters" -> compDistEntry=s.entry;
                    case "/RealOutputs/Shooter/TimeInAirSec" -> timeEntry=s.entry;
                    case "/RealOutputs/Shooter/CompensatedHubHeadingDeg" -> compHeadingEntry=s.entry;
                    case "/RealOutputs/Shooter/VelocityTowardHubMps" -> vTowEntry=s.entry;
                    case "/RealOutputs/Shooter/VelocityPerpendicularHubMps" -> vCrossEntry=s.entry;
                    case "/RealOutputs/Shooting/AutomaticFeedEnabled" -> autoFeedEntry=s.entry;
                    case "/RealOutputs/Shooting/ManualFeedOverride" -> manualEntry=s.entry;
                    case "/RealOutputs/Shooting/AimErrorDeg" -> aimErrEntry=s.entry;
                    default -> {}
                }
            } catch (IllegalArgumentException ignored) {}
        }

        var poseBuf = StructBuffer.create(Pose2d.struct);
        Pose2d pose = new Pose2d();
        double rawDist=Double.NaN, compDist=Double.NaN, time=Double.NaN, compHeading=Double.NaN, vTow=Double.NaN, vCross=Double.NaN, aimErr=Double.NaN;
        boolean autoFeed=false, manual=false;
        record Snap(double t, Pose2d pose, double rawDist, double compDist, double time, double compHeading, double vTow, double vCross, boolean autoFeed, boolean manual, double aimErr) {}
        List<Snap> snaps = new ArrayList<>();
        for (var r : records) {
            if (r.isStart() || r.isControl()) continue;
            int e = r.getEntry();
            if (e==poseEntry) pose = poseBuf.read(r.getRaw());
            else if (e==rawDistEntry) rawDist = r.getDouble();
            else if (e==compDistEntry) compDist = r.getDouble();
            else if (e==timeEntry) time = r.getDouble();
            else if (e==compHeadingEntry) compHeading = r.getDouble();
            else if (e==vTowEntry) vTow = r.getDouble();
            else if (e==vCrossEntry) vCross = r.getDouble();
            else if (e==autoFeedEntry) autoFeed = r.getBoolean();
            else if (e==manualEntry) manual = r.getBoolean();
            else if (e==aimErrEntry) aimErr = r.getDouble();
            else continue;
            snaps.add(new Snap(r.getTimestamp()/1_000_000.0, pose, rawDist, compDist, time, compHeading, vTow, vCross, autoFeed, manual, aimErr));
        }

        double[][] windows = {{22.0,23.3},{30.3,31.7},{38.6,40.2},{50.7,51.8}};
        for (double[] w : windows) {
            System.out.printf("\n=== lead consistency %.3f-%.3f ===%n", w[0], w[1]);
            double last=-1e9;
            for (var s : snaps) {
                if (s.t < w[0] || s.t > w[1]) continue;
                if (!s.autoFeed || s.manual) continue;
                if (s.t - last < 0.08) continue;
                Rotation2d rawHeading = FieldConstants.getHubFacingHeading(s.pose);
                double observedLeadDeg = wrapDeg(s.compHeading - rawHeading.getDegrees());
                double effectiveTimeSec = s.time * ShooterConstants.motionCompTimeScale() + ShooterConstants.PHASE_DELAY_SEC;
                double predictedLeadDeg = Math.toDegrees(Math.atan2(-s.vCross * effectiveTimeSec, s.rawDist - s.vTow * effectiveTimeSec));
                double leadErrorDeg = wrapDeg(observedLeadDeg - predictedLeadDeg);
                double lateralDisplacementMeters = s.vCross * effectiveTimeSec;
                double towardDisplacementMeters = s.vTow * effectiveTimeSec;
                System.out.printf(
                        "t=%.3f rawD=%.2f compD=%.2f tAir=%.3f effT=%.3f vTow=%.2f vCross=%.2f obsLead=%.1f predLead=%.1f leadErr=%.1f latDisp=%.2f towDisp=%.2f aimErr=%.2f%n",
                        s.t,
                        s.rawDist,
                        s.compDist,
                        s.time,
                        effectiveTimeSec,
                        s.vTow,
                        s.vCross,
                        observedLeadDeg,
                        predictedLeadDeg,
                        leadErrorDeg,
                        lateralDisplacementMeters,
                        towardDisplacementMeters,
                        s.aimErr);
                last = s.t;
            }
        }
    }

    private static double wrapDeg(double deg) {
        double wrapped = deg % 360.0;
        if (wrapped > 180.0) wrapped -= 360.0;
        if (wrapped <= -180.0) wrapped += 360.0;
        return wrapped;
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
