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

class AimBehaviorLogAnalyzerTest {
    @Test
    void analyzeAutoFeedAimWindows() throws Exception {
        Path logPath = Path.of("logs/akit_26-03-18_23-28-36.wpilog").toAbsolutePath();
        List<DataLogRecord> records = recordsUntilFailure(new DataLogReader(logPath.toString()));
        int poseEntry=-1, measEntry=-1, reqEntry=-1, aimReadyEntry=-1, shooterAtEntry=-1, gateEntry=-1, blockEntry=-1, stateEntry=-1,
                aimErrEntry=-1, autoFeedEntry=-1, manualEntry=-1, desiredHeadingEntry=-1, compHeadingEntry=-1, desiredRateEntry=-1,
                measuredOmegaEntry=-1, omegaCmdEntry=-1, vTowardEntry=-1, vCrossEntry=-1;
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
                    case "/RealOutputs/ShotYaw/DesiredHeadingRateRadPerSec" -> desiredRateEntry=s.entry;
                    case "/RealOutputs/ShotYaw/MeasuredOmegaRadPerSec" -> measuredOmegaEntry=s.entry;
                    case "/RealOutputs/ShotYaw/OmegaCommandRadPerSec" -> omegaCmdEntry=s.entry;
                    case "/RealOutputs/Shooter/VelocityTowardHubMps" -> vTowardEntry=s.entry;
                    case "/RealOutputs/Shooter/VelocityPerpendicularHubMps" -> vCrossEntry=s.entry;
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
        String block="", state="";
        double aimErr=Double.NaN, desiredHeading=Double.NaN, compHeading=Double.NaN, desiredRate=Double.NaN, measuredOmega=Double.NaN,
                omegaCmd=Double.NaN, vToward=Double.NaN, vCross=Double.NaN;
        record Snap(double t, Pose2d pose, ChassisSpeeds meas, ChassisSpeeds req, boolean aimReady, boolean shooterAt, boolean gate,
                    boolean autoFeed, boolean manual, String block, String state, double aimErr, double desiredHeading, double compHeading,
                    double desiredRate, double measuredOmega, double omegaCmd, double vToward, double vCross) {}
        List<Snap> snaps = new ArrayList<>();
        for (var r : records) {
            if (r.isStart() || r.isControl()) continue;
            int e = r.getEntry();
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
            else if (e==desiredHeadingEntry) desiredHeading=r.getDouble();
            else if (e==compHeadingEntry) compHeading=r.getDouble();
            else if (e==desiredRateEntry) desiredRate=r.getDouble();
            else if (e==measuredOmegaEntry) measuredOmega=r.getDouble();
            else if (e==omegaCmdEntry) omegaCmd=r.getDouble();
            else if (e==vTowardEntry) vToward=r.getDouble();
            else if (e==vCrossEntry) vCross=r.getDouble();
            else continue;
            snaps.add(new Snap(r.getTimestamp()/1_000_000.0, pose, meas, req, aimReady, shooterAt, gate, autoFeed, manual, block, state,
                    aimErr, desiredHeading, compHeading, desiredRate, measuredOmega, omegaCmd, vToward, vCross));
        }

        double[][] windows = {
                {22.0, 23.4},
                {30.3, 31.7},
                {38.6, 40.2},
                {45.9, 46.95},
                {50.7, 51.8}
        };
        for (double[] w : windows) {
            System.out.printf("\n=== window %.3f-%.3f ===%n", w[0], w[1]);
            double last=-1e9;
            for (var s : snaps) {
                if (s.t < w[0] || s.t > w[1]) continue;
                if (!s.autoFeed || s.manual) continue;
                if (s.t - last < 0.08) continue;
                System.out.printf(
                        "t=%.3f pos=(%.2f,%.2f,%.1f) reqLin=%.2f measLin=%.2f vTow=%.2f vCross=%.2f aimErr=%.2f desired=%.1f comp=%.1f dRate=%.2f measW=%.2f cmdW=%.2f aimReady=%s shooterAt=%s gate=%s block=%s state=%s%n",
                        s.t,
                        s.pose.getX(), s.pose.getY(), s.pose.getRotation().getDegrees(),
                        Math.hypot(s.req.vxMetersPerSecond, s.req.vyMetersPerSecond),
                        Math.hypot(s.meas.vxMetersPerSecond, s.meas.vyMetersPerSecond),
                        s.vToward, s.vCross, s.aimErr, s.desiredHeading, s.compHeading, s.desiredRate, s.measuredOmega, s.omegaCmd,
                        s.aimReady, s.shooterAt, s.gate, s.block, s.state);
                last=s.t;
            }
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
