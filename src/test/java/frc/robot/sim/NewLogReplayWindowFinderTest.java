package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.util.struct.StructBuffer;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.Test;

class NewLogReplayWindowFinderTest {
    private static final Path LOG_PATH = Path.of("logs/akit_26-03-19_00-08-50.wpilog").toAbsolutePath();

    @Test
    void findCandidateReplayWindows() throws Exception {
        List<Snap> snaps = read(LOG_PATH);
        for (int i = 1; i < snaps.size(); i++) {
            Snap prev = snaps.get(i - 1);
            Snap cur = snaps.get(i);
            if (!(prev.stickMag() < 0.05 && cur.stickMag() >= 0.40)) continue;
            if (!cur.rightBumper || !cur.autoFeed || cur.manualFeed) continue;
            double start = cur.t - 1.5;
            double end = cur.t + 2.5;
            int movingGate = 0;
            double maxErrWhenGate = 0.0;
            for (Snap s : snaps) {
                if (s.t < start || s.t > end) continue;
                if (s.gate && s.speed() >= 0.25) {
                    movingGate++;
                    maxErrWhenGate = Math.max(maxErrWhenGate, Math.abs(s.aimErrDeg));
                }
            }
            if (movingGate == 0) continue;
            System.out.printf(Locale.US,
                    "candidate onset=%.3f start=%.3f end=%.3f movingGate=%d maxErrGate=%.2f pose=(%.2f,%.2f,%.1f) rb=%s rt=%.2f state=%s block=%s%n",
                    cur.t, start, end, movingGate, maxErrWhenGate,
                    cur.pose.getX(), cur.pose.getY(), cur.pose.getRotation().getDegrees(),
                    cur.rightBumper, cur.rightTrigger, cur.state, cur.blockReason);
        }
    }

    private record Snap(
            double t,
            Pose2d pose,
            float leftX,
            float leftY,
            float rightX,
            float rightTrigger,
            boolean rightBumper,
            boolean autoFeed,
            boolean manualFeed,
            boolean gate,
            double aimErrDeg,
            String state,
            String blockReason) {
        double stickMag() { return Math.hypot(leftX, leftY); }
        double speed() { return stickMag(); }
    }

    private static List<Snap> read(Path logPath) throws Exception {
        DataLogReader reader = new DataLogReader(logPath.toString());
        StructBuffer<Pose2d> poseBuffer = StructBuffer.create(Pose2d.struct);
        int axisEntry=-1, buttonEntry=-1, poseEntry=-1, autoFeedEntry=-1, manualFeedEntry=-1, gateEntry=-1, aimErrEntry=-1, stateEntry=-1, blockEntry=-1;
        for (DataLogRecord record : recordsUntilFailure(reader)) {
            if (!record.isStart()) continue;
            try {
                var s = record.getStartData();
                switch (s.name) {
                    case "/DriverStation/Joystick0/AxisValues" -> axisEntry = s.entry;
                    case "/DriverStation/Joystick0/ButtonValues" -> buttonEntry = s.entry;
                    case "/RealOutputs/Odometry/Robot" -> poseEntry = s.entry;
                    case "/RealOutputs/Shooting/AutomaticFeedEnabled" -> autoFeedEntry = s.entry;
                    case "/RealOutputs/Shooting/ManualFeedOverride" -> manualFeedEntry = s.entry;
                    case "/RealOutputs/Shooting/GateOpen" -> gateEntry = s.entry;
                    case "/RealOutputs/Shooting/AimErrorDeg" -> aimErrEntry = s.entry;
                    case "/RealOutputs/Shooting/State" -> stateEntry = s.entry;
                    case "/RealOutputs/Shooting/BlockReason" -> blockEntry = s.entry;
                    default -> {}
                }
            } catch (IllegalArgumentException ignored) {}
        }
        float[] axes = new float[0];
        long buttons = 0;
        Pose2d pose = new Pose2d();
        boolean autoFeed=false, manualFeed=false, gate=false;
        double aimErrDeg = Double.NaN;
        String state="", blockReason="";
        List<Snap> snaps = new ArrayList<>();
        for (DataLogRecord record : recordsUntilFailure(new DataLogReader(logPath.toString()))) {
            if (record.isStart() || record.isControl()) continue;
            int e = record.getEntry();
            if (e == axisEntry) axes = record.getFloatArray();
            else if (e == buttonEntry) buttons = record.getInteger();
            else if (e == poseEntry) pose = poseBuffer.read(record.getRaw());
            else if (e == autoFeedEntry) autoFeed = record.getBoolean();
            else if (e == manualFeedEntry) manualFeed = record.getBoolean();
            else if (e == gateEntry) gate = record.getBoolean();
            else if (e == aimErrEntry) aimErrDeg = record.getDouble();
            else if (e == stateEntry) state = record.getString();
            else if (e == blockEntry) blockReason = record.getString();
            else continue;
            float leftX = axes.length > 0 ? axes[0] : 0.0f;
            float leftY = axes.length > 1 ? axes[1] : 0.0f;
            float rightTrigger = axes.length > 3 ? axes[3] : 0.0f;
            float rightX = axes.length > 4 ? axes[4] : 0.0f;
            boolean rightBumper = (buttons & (1L << 5)) != 0;
            snaps.add(new Snap(record.getTimestamp() / 1_000_000.0, pose, leftX, leftY, rightX, rightTrigger, rightBumper, autoFeed, manualFeed, gate, aimErrDeg, state, blockReason));
        }
        return snaps;
    }

    private static List<DataLogRecord> recordsUntilFailure(DataLogReader reader) {
        List<DataLogRecord> records = new ArrayList<>();
        Iterator<DataLogRecord> iterator = reader.iterator();
        while (true) {
            try {
                if (!iterator.hasNext()) break;
                records.add(iterator.next());
            } catch (IllegalArgumentException exception) {
                break;
            }
        }
        return records;
    }
}
