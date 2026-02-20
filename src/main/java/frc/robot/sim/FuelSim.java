package frc.robot.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

/** Simulates a small pooled set of FUEL game pieces for visualization. */
public final class FuelSim {
    private static final int FUEL_COUNT = 50;
    private static final double GRAVITY_METERS_PER_SEC2 = 9.80665;
    private static final double FUEL_DIAMETER_METERS = Units.inchesToMeters(5.9);

    private static final double SPAWN_PERIOD_SEC = 0.12;
    private static final double SHOOTER_WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);
    private static final double SHOOTER_SPEED_EFFICIENCY = 0.85;
    private static final double MIN_MUZZLE_SPEED_METERS_PER_SEC = 7.0;
    private static final double MAX_MUZZLE_SPEED_METERS_PER_SEC = 25.0;
    private static final double MUZZLE_OFFSET_X_METERS = 0.32;
    private static final double MUZZLE_OFFSET_Y_METERS = 0.0;
    private static final double MUZZLE_HEIGHT_METERS = 0.68;
    private static final double MAX_HOOD_ANGLE_RAD = Units.degreesToRadians(90.0);

    private final double[] xMeters = new double[FUEL_COUNT];
    private final double[] yMeters = new double[FUEL_COUNT];
    private final double[] zMeters = new double[FUEL_COUNT];
    private final double[] vxMetersPerSec = new double[FUEL_COUNT];
    private final double[] vyMetersPerSec = new double[FUEL_COUNT];
    private final double[] vzMetersPerSec = new double[FUEL_COUNT];
    private final boolean[] active = new boolean[FUEL_COUNT];
    private final Pose3d[] publishedPoses = new Pose3d[FUEL_COUNT];

    private int nextSpawnIndex = 0;
    private double spawnAccumulatorSec = 0.0;
    private Pose2d lastRobotPose = null;

    public FuelSim() {
        for (int i = 0; i < FUEL_COUNT; i++) {
            publishedPoses[i] = new Pose3d();
        }
    }

    public void update(
            Pose2d robotPose,
            double shooterRpm,
            double hoodAngleRad,
            boolean shooterActive,
            double dtSec) {
        if (robotPose == null || !Double.isFinite(dtSec) || dtSec <= 0.0) {
            publish();
            return;
        }

        double clampedDtSec = Math.max(1e-4, Math.min(dtSec, 0.05));
        RobotVelocity robotVelocity = computeRobotVelocity(robotPose, clampedDtSec);

        if (shooterActive) {
            spawnAccumulatorSec += clampedDtSec;
            while (spawnAccumulatorSec >= SPAWN_PERIOD_SEC) {
                spawnAccumulatorSec -= SPAWN_PERIOD_SEC;
                spawnFuel(robotPose, robotVelocity, shooterRpm, hoodAngleRad);
            }
        }

        for (int i = 0; i < FUEL_COUNT; i++) {
            if (!active[i]) {
                continue;
            }

            double vx = vxMetersPerSec[i];
            double vy = vyMetersPerSec[i];
            double vz = vzMetersPerSec[i];
            vz -= GRAVITY_METERS_PER_SEC2 * clampedDtSec;
            xMeters[i] += vx * clampedDtSec;
            yMeters[i] += vy * clampedDtSec;
            zMeters[i] += vz * clampedDtSec;
            vxMetersPerSec[i] = vx;
            vyMetersPerSec[i] = vy;
            vzMetersPerSec[i] = vz;

            if (zMeters[i] < 0.0) {
                resetFuel(i);
            }
        }

        publish();
    }

    private RobotVelocity computeRobotVelocity(Pose2d currentPose, double dtSec) {
        if (lastRobotPose == null) {
            lastRobotPose = currentPose;
            return new RobotVelocity(0.0, 0.0, 0.0);
        }

        double vx = (currentPose.getX() - lastRobotPose.getX()) / dtSec;
        double vy = (currentPose.getY() - lastRobotPose.getY()) / dtSec;
        double dtheta = MathUtil.angleModulus(
                currentPose.getRotation().getRadians() - lastRobotPose.getRotation().getRadians());
        double omega = dtheta / dtSec;

        lastRobotPose = currentPose;
        return new RobotVelocity(vx, vy, omega);
    }

    private void spawnFuel(Pose2d robotPose, RobotVelocity robotVelocity, double shooterRpm, double hoodAngleRad) {
        int index = nextSpawnIndex;
        nextSpawnIndex = (nextSpawnIndex + 1) % FUEL_COUNT;

        double yaw = robotPose.getRotation().getRadians();
        Translation2d muzzleOffsetRobot = new Translation2d(MUZZLE_OFFSET_X_METERS, MUZZLE_OFFSET_Y_METERS);
        Translation2d muzzleOffsetField = muzzleOffsetRobot.rotateBy(robotPose.getRotation());

        double spawnX = robotPose.getX() + muzzleOffsetField.getX();
        double spawnY = robotPose.getY() + muzzleOffsetField.getY();
        double spawnZ = MUZZLE_HEIGHT_METERS;

        double tangentialVx = -robotVelocity.omegaRadPerSec() * muzzleOffsetField.getY();
        double tangentialVy = robotVelocity.omegaRadPerSec() * muzzleOffsetField.getX();

        double hood = MathUtil.clamp(hoodAngleRad, 0.0, MAX_HOOD_ANGLE_RAD);
        double muzzleSpeed = computeMuzzleSpeedMetersPerSec(shooterRpm);
        double horizontalSpeed = muzzleSpeed * Math.cos(hood);
        double launchVx = robotVelocity.vxMetersPerSec() + tangentialVx + horizontalSpeed * Math.cos(yaw);
        double launchVy = robotVelocity.vyMetersPerSec() + tangentialVy + horizontalSpeed * Math.sin(yaw);
        double launchVz = muzzleSpeed * Math.sin(hood);

        xMeters[index] = spawnX;
        yMeters[index] = spawnY;
        zMeters[index] = spawnZ;
        vxMetersPerSec[index] = launchVx;
        vyMetersPerSec[index] = launchVy;
        vzMetersPerSec[index] = launchVz;
        active[index] = true;
    }

    private static double computeMuzzleSpeedMetersPerSec(double shooterRpm) {
        double wheelRadsPerSec = Math.abs(shooterRpm) * (2.0 * Math.PI / 60.0);
        double wheelSurfaceSpeed = wheelRadsPerSec * SHOOTER_WHEEL_RADIUS_METERS * SHOOTER_SPEED_EFFICIENCY;
        return MathUtil.clamp(
                wheelSurfaceSpeed,
                MIN_MUZZLE_SPEED_METERS_PER_SEC,
                MAX_MUZZLE_SPEED_METERS_PER_SEC);
    }

    private void resetFuel(int index) {
        xMeters[index] = 0.0;
        yMeters[index] = 0.0;
        zMeters[index] = 0.0;
        vxMetersPerSec[index] = 0.0;
        vyMetersPerSec[index] = 0.0;
        vzMetersPerSec[index] = 0.0;
        active[index] = false;
    }

    private void publish() {
        int activeCount = 0;
        for (int i = 0; i < FUEL_COUNT; i++) {
            if (active[i]) {
                activeCount++;
                publishedPoses[i] = new Pose3d(xMeters[i], yMeters[i], zMeters[i], new Rotation3d());
            } else {
                publishedPoses[i] = new Pose3d();
            }
        }

        Logger.recordOutput("sim/FUEL/poses", publishedPoses);
        Logger.recordOutput("sim/FUEL/activeCount", activeCount);
        Logger.recordOutput("sim/FUEL/diameterMeters", FUEL_DIAMETER_METERS);
    }

    private record RobotVelocity(double vxMetersPerSec, double vyMetersPerSec, double omegaRadPerSec) {}
}
