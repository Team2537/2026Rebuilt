package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.util.struct.StructBuffer;
import frc.robot.util.FieldConstants;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import org.junit.jupiter.api.Test;

class AimGeometryLogAnalyzerTest {
    @Test
    void compareRawVsCompensatedHeading() throws Exception {
        Path logPath = Path.of("logs/akit_26-03-18_23-28-36.wpilog").toAbsolutePath();
        List<DataLogRecord> records = recordsUntilFailure(new DataLogReader(logPath.toString()));
        int poseEntry=-1, compHeadingEntry=-1, aimErrEntry=-1, autoFeedEntry=-1, manualEntry=-1, stateEntry=-1;
        for (var record : records) {
            if (!record.isStart()) continue;
            try {
                var s = record.getStartData();
                switch (s.name) {
                    case "/RealOutputs/Odometry/Robot" -> poseEntry=s.entry;
                    case "/RealOutputs/Shooter/CompensatedHubHeadingDeg" -> compHeadingEntry=s.entry;
                    case "/RealOutputs/Shooting/AimErrorDeg" -> aimErrEntry=s.entry;
                    case "/RealOutputs/Shooting/AutomaticFeedEnabled" -> autoFeedEntry=s.entry;
                    case "/RealOutputs/Shooting/ManualFeedOverride" -> manualEntry=s.entry;
                    case "/RealOutputs/Shooting/State" -> stateEntry=s.entry;
                    default -> {}
                }
            } catch (IllegalArgumentException ignored) {}
        }
        var poseBuf = StructBuffer.create(Pose2d.struct);
        Pose2d pose = new Pose2d();
        double compHeadingDeg = Double.NaN;
        double aimErrDeg = Double.NaN;
        boolean autoFeed = false, manual = false;
        String state = "";
        record Snap(double t, Pose2d pose, double compHeadingDeg, double aimErrDeg, boolean autoFeed, boolean manual, String state) {}
        List<Snap> snaps = new ArrayList<>();
        for (var r : records) {
            if (r.isStart() || r.isControl()) continue;
            int e = r.getEntry();
            if (e==poseEntry) pose = poseBuf.read(r.getRaw());
            else if (e==compHeadingEntry) compHeadingDeg = r.getDouble();
            else if (e==aimErrEntry) aimErrDeg = r.getDouble();
            else if (e==autoFeedEntry) autoFeed = r.getBoolean();
            else if (e==manualEntry) manual = r.getBoolean();
            else if (e==stateEntry) state = r.getString();
            else continue;
            snaps.add(new Snap(r.getTimestamp()/1_000_000.0, pose, compHeadingDeg, aimErrDeg, autoFeed, manual, state));
        }

        double[][] windows = {{22.0,23.3},{30.3,31.7},{38.6,40.2},{50.7,51.8}};
        for (double[] w : windows) {
            System.out.printf("\n=== geometry %.3f-%.3f ===%n", w[0], w[1]);
            double last=-1e9;
            for (var s : snaps) {
                if (s.t < w[0] || s.t > w[1]) continue;
                if (!s.autoFeed || s.manual) continue;
                if (s.t-last < 0.08) continue;
                Rotation2d rawHeading = FieldConstants.getHubFacingHeading(s.pose);
                double leadDeg = angleDeg(s.compHeadingDeg - rawHeading.getDegrees());
                double desiredRobotHeadingDeg = angleDeg(s.compHeadingDeg + 180.0);
                double actualRobotHeadingDeg = s.pose.getRotation().getDegrees();
                double actualMinusRawDeg = angleDeg(actualRobotHeadingDeg - angleDeg(rawHeading.getDegrees()+180.0));
                System.out.printf(
                        "t=%.3f raw=%.1f comp=%.1f lead=%.1f desiredRobot=%.1f actualRobot=%.1f aimErr=%.2f actual-vs-rawRobot=%.1f state=%s%n",
                        s.t,
                        rawHeading.getDegrees(),
                        s.compHeadingDeg,
                        leadDeg,
                        desiredRobotHeadingDeg,
                        actualRobotHeadingDeg,
                        s.aimErrDeg,
                        actualMinusRawDeg,
                        s.state);
                last=s.t;
            }
        }
    }

    private static double angleDeg(double deg) {
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
