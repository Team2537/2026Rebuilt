package frc.robot.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.util.struct.StructBuffer;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;
import org.junit.jupiter.api.Test;

class ShootOnMoveLogAnalyzerTest {
    private static final String LOG_PATH_PROPERTY = "shootOnMoveDiag.wpilog";
    private static final Path DEFAULT_LOG_PATH = Path.of("logs/akit_26-03-18_22-40-46.wpilog").toAbsolutePath();

    @Test
    void summarizeShootOnMoveBehavior() throws Exception {
        Path logPath = configuredLogPath();
        System.out.println("Analyzing log: " + logPath);
        DataLogReader reader = new DataLogReader(logPath.toString());
        int axisEntry = -1;
        int buttonEntry = -1;
        int poseEntry = -1;
        int measuredEntry = -1;
        int requestedEntry = -1;
        int aimErrEntry = -1;
        int gateEntry = -1;
        int shootStateEntry = -1;
        int blockReasonEntry = -1;
        int desiredHeadingEntry = -1;
        int compHeadingEntry = -1;
        int distanceEntry = -1;
        int rtModeEntry = -1;
        int eventEntry = -1;

        for (DataLogRecord record : recordsUntilFailure(reader)) {
            if (!record.isStart()) continue;
            DataLogRecord.StartRecordData start;
            try {
                start = record.getStartData();
            } catch (IllegalArgumentException ignored) {
                continue;
            }
            switch (start.name) {
                case "/DriverStation/Joystick0/AxisValues" -> axisEntry = start.entry;
                case "/DriverStation/Joystick0/ButtonValues" -> buttonEntry = start.entry;
                case "/RealOutputs/Odometry/Robot" -> poseEntry = start.entry;
                case "/RealOutputs/SwerveChassisSpeeds/Measured" -> measuredEntry = start.entry;
                case "/RealOutputs/SwerveChassisSpeeds/Requested" -> requestedEntry = start.entry;
                case "/RealOutputs/Shooting/AimErrorDeg" -> aimErrEntry = start.entry;
                case "/RealOutputs/Shooting/GateOpen" -> gateEntry = start.entry;
                case "/RealOutputs/Shooting/State" -> shootStateEntry = start.entry;
                case "/RealOutputs/Shooting/BlockReason" -> blockReasonEntry = start.entry;
                case "/RealOutputs/Shooting/DesiredRobotHeadingDeg" -> desiredHeadingEntry = start.entry;
                case "/RealOutputs/Shooter/CompensatedHubHeadingDeg" -> compHeadingEntry = start.entry;
                case "/RealOutputs/Shooting/DistanceMeters" -> distanceEntry = start.entry;
                case "/RealOutputs/Shooting/RightTriggerMode" -> rtModeEntry = start.entry;
                case "/RealOutputs/Commands/lastEvent" -> eventEntry = start.entry;
                default -> {}
            }
        }

        StructBuffer<Pose2d> poseBuffer = StructBuffer.create(Pose2d.struct);
        StructBuffer<ChassisSpeeds> speedsBuffer = StructBuffer.create(ChassisSpeeds.struct);

        float[] axes = new float[0];
        long buttons = 0L;
        Pose2d pose = new Pose2d();
        ChassisSpeeds measured = new ChassisSpeeds();
        ChassisSpeeds requested = new ChassisSpeeds();
        double aimErrDeg = Double.NaN;
        boolean gateOpen = false;
        String shootState = "";
        String blockReason = "";
        double desiredHeadingDeg = Double.NaN;
        double compHeadingDeg = Double.NaN;
        double distanceMeters = Double.NaN;
        String rightTriggerMode = "";
        String lastEvent = "";

        List<Snapshot> snapshots = new ArrayList<>();
        for (DataLogRecord record : recordsUntilFailure(new DataLogReader(logPath.toString()))) {
            if (record.isStart() || record.isControl()) continue;
            int entry = record.getEntry();
            if (entry == axisEntry) {
                axes = record.getFloatArray();
            } else if (entry == buttonEntry) {
                buttons = record.getInteger();
            } else if (entry == poseEntry) {
                pose = poseBuffer.read(record.getRaw());
            } else if (entry == measuredEntry) {
                measured = speedsBuffer.read(record.getRaw());
            } else if (entry == requestedEntry) {
                requested = speedsBuffer.read(record.getRaw());
            } else if (entry == aimErrEntry) {
                aimErrDeg = record.getDouble();
            } else if (entry == gateEntry) {
                gateOpen = record.getBoolean();
            } else if (entry == shootStateEntry) {
                shootState = record.getString();
            } else if (entry == blockReasonEntry) {
                blockReason = record.getString();
            } else if (entry == desiredHeadingEntry) {
                desiredHeadingDeg = record.getDouble();
            } else if (entry == compHeadingEntry) {
                compHeadingDeg = record.getDouble();
            } else if (entry == distanceEntry) {
                distanceMeters = record.getDouble();
            } else if (entry == rtModeEntry) {
                rightTriggerMode = record.getString();
            } else if (entry == eventEntry) {
                lastEvent = record.getString();
            } else {
                continue;
            }

            double t = record.getTimestamp() / 1_000_000.0;
            float leftX = axes.length > 0 ? axes[0] : 0.0f;
            float leftY = axes.length > 1 ? axes[1] : 0.0f;
            float rightTrigger = axes.length > 3 ? axes[3] : 0.0f;
            float rightX = axes.length > 4 ? axes[4] : 0.0f;
            boolean rightBumper = (buttons & (1L << 5)) != 0;
            snapshots.add(new Snapshot(
                    t,
                    leftX,
                    leftY,
                    rightTrigger,
                    rightX,
                    rightBumper,
                    pose,
                    measured,
                    requested,
                    aimErrDeg,
                    gateOpen,
                    shootState,
                    blockReason,
                    desiredHeadingDeg,
                    compHeadingDeg,
                    distanceMeters,
                    rightTriggerMode,
                    lastEvent));
        }

        snapshots.sort(Comparator.comparingDouble(Snapshot::t));

        double prevRt = 0.0;
        boolean printedWindow = false;
        for (int i = 0; i < snapshots.size(); i++) {
            Snapshot s = snapshots.get(i);
            if (prevRt <= 0.5 && s.rightTrigger > 0.5) {
                System.out.printf("\n=== right-trigger press at %.3fs mode=%s state=%s pose=(%.2f, %.2f, %.1fdeg) ===%n",
                        s.t,
                        s.rightTriggerMode,
                        s.shootState,
                        s.pose.getX(),
                        s.pose.getY(),
                        s.pose.getRotation().getDegrees());
                printWindow(snapshots, s.t - 0.4, s.t + 4.0);
                printedWindow = true;
            }
            prevRt = s.rightTrigger;
        }

        if (!printedWindow) {
            System.out.println("No right-trigger press detected.");
        }

        System.out.println("\n=== translation-onset while shooting ===");
        for (int i = 1; i < snapshots.size(); i++) {
            Snapshot prev = snapshots.get(i - 1);
            Snapshot s = snapshots.get(i);
            double prevTrans = Math.hypot(prev.leftX, prev.leftY);
            double trans = Math.hypot(s.leftX, s.leftY);
            boolean rtHeld = s.rightTrigger > 0.5;
            if (rtHeld && prevTrans < 0.05 && trans > 0.20) {
                System.out.printf(
                        "onset %.3fs left=(%.2f, %.2f) rb=%s req=(%.2f, %.2f, %.2f) meas=(%.2f, %.2f, %.2f) aimErr=%.2f gate=%s state=%s block=%s heading=%.1f desired=%.1f comp=%.1f pose=(%.2f, %.2f, %.1fdeg) event=%s%n",
                        s.t,
                        s.leftX,
                        s.leftY,
                        s.rightBumper,
                        s.requested.vxMetersPerSecond,
                        s.requested.vyMetersPerSecond,
                        s.requested.omegaRadiansPerSecond,
                        s.measured.vxMetersPerSecond,
                        s.measured.vyMetersPerSecond,
                        s.measured.omegaRadiansPerSecond,
                        s.aimErrDeg,
                        s.gateOpen,
                        s.shootState,
                        s.blockReason,
                        s.pose.getRotation().getDegrees(),
                        s.desiredHeadingDeg,
                        s.compHeadingDeg,
                        s.pose.getX(),
                        s.pose.getY(),
                        s.pose.getRotation().getDegrees(),
                        s.lastEvent);
                printWindow(snapshots, s.t - 0.5, s.t + 1.5);
            }
        }
    }

    private static Path configuredLogPath() {
        String raw = System.getProperty(LOG_PATH_PROPERTY);
        if (raw == null || raw.isBlank()) {
            return DEFAULT_LOG_PATH;
        }
        return Path.of(raw).toAbsolutePath();
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
            } catch (IllegalArgumentException exception) {
                System.out.println("Stopped log scan after parser failure: " + exception.getMessage());
                break;
            }
        }
        return records;
    }

    private static void printWindow(List<Snapshot> snapshots, double startSec, double endSec) {
        double lastPrinted = Double.NEGATIVE_INFINITY;
        for (Snapshot s : snapshots) {
            if (s.t < startSec || s.t > endSec) continue;
            if (s.t - lastPrinted < 0.10) continue;
            double requestedLinear = Math.hypot(s.requested.vxMetersPerSecond, s.requested.vyMetersPerSecond);
            double measuredLinear = Math.hypot(s.measured.vxMetersPerSecond, s.measured.vyMetersPerSecond);
            double desiredErrorDeg = Double.isFinite(s.desiredHeadingDeg)
                    ? Math.toDegrees(MathUtil.angleModulus(Math.toRadians(s.desiredHeadingDeg - s.pose.getRotation().getDegrees())))
                    : Double.NaN;
            System.out.printf(
                    "t=%.3f lt=(%.2f,%.2f) rt=%.2f rb=%s rx=%.2f reqLin=%.2f measLin=%.2f reqW=%.2f measW=%.2f aimErr=%.2f desiredErr=%.2f gate=%s state=%s block=%s pose=(%.2f,%.2f,%.1f) dist=%.2f event=%s%n",
                    s.t,
                    s.leftX,
                    s.leftY,
                    s.rightTrigger,
                    s.rightBumper,
                    s.rightX,
                    requestedLinear,
                    measuredLinear,
                    s.requested.omegaRadiansPerSecond,
                    s.measured.omegaRadiansPerSecond,
                    s.aimErrDeg,
                    desiredErrorDeg,
                    s.gateOpen,
                    s.shootState,
                    s.blockReason,
                    s.pose.getX(),
                    s.pose.getY(),
                    s.pose.getRotation().getDegrees(),
                    s.distanceMeters,
                    s.lastEvent);
            lastPrinted = s.t;
        }
    }

    private record Snapshot(
            double t,
            float leftX,
            float leftY,
            float rightTrigger,
            float rightX,
            boolean rightBumper,
            Pose2d pose,
            ChassisSpeeds measured,
            ChassisSpeeds requested,
            double aimErrDeg,
            boolean gateOpen,
            String shootState,
            String blockReason,
            double desiredHeadingDeg,
            double compHeadingDeg,
            double distanceMeters,
            String rightTriggerMode,
            String lastEvent) {}
}
