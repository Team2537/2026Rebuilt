package frc.robot.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.FieldConstants;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/** Simulates a small pooled set of FUEL game pieces for visualization. */
public final class FuelSim {
    private static final int FUEL_COUNT = 50;
    private static final double GRAVITY_METERS_PER_SEC2 = 9.80665;
    private static final double FUEL_DIAMETER_METERS = Units.inchesToMeters(5.9);

    private static final double SPAWN_PERIOD_SEC = 0.16;
    private static final double SHOOTER_WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);
    private static final double SHOOTER_SPEED_EFFICIENCY = 0.85;
    private static final double MIN_MUZZLE_SPEED_METERS_PER_SEC = 7.0;
    private static final double MAX_MUZZLE_SPEED_METERS_PER_SEC = 25.0;
    private static final double MUZZLE_OFFSET_X_METERS = -0.1524;
    private static final double MUZZLE_OFFSET_Y_METERS = 0.0;
    private static final double MUZZLE_HEIGHT_METERS = 0.68;
    private static final double MAX_HOOD_ANGLE_RAD = Units.degreesToRadians(90.0);
    private static final int RECENT_MISS_HISTORY_SIZE = 20;

    public static final double DESCENT_ACCURACY_HEIGHT_METERS = 1.83;
    public static final double IDEAL_DESCENT_MISS_DISTANCE_METERS = Units.inchesToMeters(6.0);

    private final FuelState[] fuelStates = new FuelState[FUEL_COUNT];
    private final Pose3d[] publishedPoses = new Pose3d[FUEL_COUNT];
    private final ArrayDeque<DescentCrossingSample> pendingDescentCrossingSamples = new ArrayDeque<>();
    private final double[] recentDescentMissMeters = new double[RECENT_MISS_HISTORY_SIZE];

    private int nextSpawnIndex = 0;
    private double spawnAccumulatorSec = 0.0;
    private Pose2d lastRobotPose = null;
    private double simTimeSec = 0.0;
    private int recentDescentMissCount = 0;
    private int recentDescentMissInsertIndex = 0;
    private long totalDescentCrossingSamples = 0L;
    private long descentCrossingSamplesWithinIdeal = 0L;
    private double latestDescentMissMeters = Double.NaN;
    private double minDescentMissMeters = Double.NaN;
    private double maxDescentMissMeters = Double.NaN;
    private double sumDescentMissMeters = 0.0;

    public record DescentCrossingSample(
            double timestampSec,
            double xMeters,
            double yMeters,
            double missDistanceMeters,
            double signedMissDistanceMeters,
            double alongVelocityMissMeters,
            double crossVelocityMissMeters,
            double velocityToTargetAngleDeg) {}

    public FuelSim() {
        for (int i = 0; i < FUEL_COUNT; i++) {
            fuelStates[i] = new FuelState();
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
        double cycleStartTimeSec = simTimeSec;
        RobotVelocity robotVelocity = computeRobotVelocity(robotPose, clampedDtSec);

        if (shooterActive) {
            spawnAccumulatorSec += clampedDtSec;
            while (spawnAccumulatorSec >= SPAWN_PERIOD_SEC) {
                spawnAccumulatorSec -= SPAWN_PERIOD_SEC;
                spawnFuel(robotPose, robotVelocity, shooterRpm, hoodAngleRad);
            }
        }

        for (FuelState fuel : fuelStates) {
            if (!fuel.active) {
                continue;
            }

            double previousX = fuel.xMeters;
            double previousY = fuel.yMeters;
            double previousZ = fuel.zMeters;
            double previousVz = fuel.vzMetersPerSec;

            fuel.vzMetersPerSec -= GRAVITY_METERS_PER_SEC2 * clampedDtSec;
            fuel.xMeters += fuel.vxMetersPerSec * clampedDtSec;
            fuel.yMeters += fuel.vyMetersPerSec * clampedDtSec;
            fuel.zMeters += fuel.vzMetersPerSec * clampedDtSec;

            recordDescentCrossingSample(
                    cycleStartTimeSec,
                    clampedDtSec,
                    previousX,
                    previousY,
                    previousZ,
                    previousVz,
                    fuel.xMeters,
                    fuel.yMeters,
                    fuel.zMeters,
                    fuel.vzMetersPerSec,
                    fuel.vxMetersPerSec,
                    fuel.vyMetersPerSec);

            if (fuel.zMeters < 0.0) {
                resetFuel(fuel);
            }
        }

        simTimeSec = cycleStartTimeSec + clampedDtSec;
        publish();
    }

    public List<DescentCrossingSample> drainDescentCrossingSamples() {
        List<DescentCrossingSample> samples = new ArrayList<>(pendingDescentCrossingSamples);
        pendingDescentCrossingSamples.clear();
        return samples;
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
        FuelState fuel = fuelStates[nextSpawnIndex];
        nextSpawnIndex = (nextSpawnIndex + 1) % FUEL_COUNT;

        double yaw = robotPose.getRotation().getRadians() + Math.PI;
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

        fuel.xMeters = spawnX;
        fuel.yMeters = spawnY;
        fuel.zMeters = spawnZ;
        fuel.vxMetersPerSec = launchVx;
        fuel.vyMetersPerSec = launchVy;
        fuel.vzMetersPerSec = launchVz;
        fuel.active = true;
    }

    private static double computeMuzzleSpeedMetersPerSec(double shooterRpm) {
        double wheelRadsPerSec = Math.abs(shooterRpm) * (2.0 * Math.PI / 60.0);
        double wheelSurfaceSpeed = wheelRadsPerSec * SHOOTER_WHEEL_RADIUS_METERS * SHOOTER_SPEED_EFFICIENCY;
        return MathUtil.clamp(
                wheelSurfaceSpeed,
                MIN_MUZZLE_SPEED_METERS_PER_SEC,
                MAX_MUZZLE_SPEED_METERS_PER_SEC);
    }

    private static void resetFuel(FuelState fuel) {
        fuel.xMeters = 0.0;
        fuel.yMeters = 0.0;
        fuel.zMeters = 0.0;
        fuel.vxMetersPerSec = 0.0;
        fuel.vyMetersPerSec = 0.0;
        fuel.vzMetersPerSec = 0.0;
        fuel.active = false;
    }

    private void recordDescentCrossingSample(
            double cycleStartTimeSec,
            double dtSec,
            double previousX,
            double previousY,
            double previousZ,
            double previousVz,
            double currentX,
            double currentY,
            double currentZ,
            double currentVz,
            double horizontalVelocityXMps,
            double horizontalVelocityYMps) {
        if (!(previousZ > DESCENT_ACCURACY_HEIGHT_METERS
                && currentZ <= DESCENT_ACCURACY_HEIGHT_METERS
                && currentZ < previousZ)) {
            return;
        }

        Translation2d hubTarget = FieldConstants.getHubTargetTranslation();
        if (hubTarget == null) {
            return;
        }

        double zDelta = previousZ - currentZ;
        if (zDelta <= 1e-9) {
            return;
        }

        double alpha = MathUtil.clamp((previousZ - DESCENT_ACCURACY_HEIGHT_METERS) / zDelta, 0.0, 1.0);
        double vzAtCrossing = previousVz + (currentVz - previousVz) * alpha;
        if (vzAtCrossing >= 0.0) {
            return;
        }

        double xAtCrossing = previousX + (currentX - previousX) * alpha;
        double yAtCrossing = previousY + (currentY - previousY) * alpha;
        double crossingTimeSec = cycleStartTimeSec + alpha * dtSec;
        Translation2d crossingPoint = new Translation2d(xAtCrossing, yAtCrossing);
        Translation2d toTarget = hubTarget.minus(crossingPoint);
        double missDistanceMeters = toTarget.getNorm();
        double signedMissDistanceMeters = 0.0;
        double alongVelocityMissMeters = 0.0;
        double crossVelocityMissMeters = 0.0;
        double velocityToTargetAngleDeg = Double.NaN;
        double planarSpeedMps = Math.hypot(horizontalVelocityXMps, horizontalVelocityYMps);
        if (planarSpeedMps > 1e-6) {
            double velocityUnitX = horizontalVelocityXMps / planarSpeedMps;
            double velocityUnitY = horizontalVelocityYMps / planarSpeedMps;
            double projectionMetersPerSec =
                    toTarget.getX() * horizontalVelocityXMps + toTarget.getY() * horizontalVelocityYMps;
            alongVelocityMissMeters = toTarget.getX() * velocityUnitX + toTarget.getY() * velocityUnitY;
            crossVelocityMissMeters = velocityUnitX * toTarget.getY() - velocityUnitY * toTarget.getX();
            velocityToTargetAngleDeg =
                    Units.radiansToDegrees(Math.atan2(crossVelocityMissMeters, alongVelocityMissMeters));
            signedMissDistanceMeters = missDistanceMeters * Math.signum(projectionMetersPerSec);
        }

        pendingDescentCrossingSamples.addLast(new DescentCrossingSample(
                crossingTimeSec,
                xAtCrossing,
                yAtCrossing,
                missDistanceMeters,
                signedMissDistanceMeters,
                alongVelocityMissMeters,
                crossVelocityMissMeters,
                velocityToTargetAngleDeg));
        latestDescentMissMeters = missDistanceMeters;
        if (!Double.isFinite(minDescentMissMeters) || missDistanceMeters < minDescentMissMeters) {
            minDescentMissMeters = missDistanceMeters;
        }
        if (!Double.isFinite(maxDescentMissMeters) || missDistanceMeters > maxDescentMissMeters) {
            maxDescentMissMeters = missDistanceMeters;
        }
        sumDescentMissMeters += missDistanceMeters;
        totalDescentCrossingSamples++;
        if (missDistanceMeters <= IDEAL_DESCENT_MISS_DISTANCE_METERS) {
            descentCrossingSamplesWithinIdeal++;
        }

        recentDescentMissMeters[recentDescentMissInsertIndex] = missDistanceMeters;
        recentDescentMissInsertIndex = (recentDescentMissInsertIndex + 1) % RECENT_MISS_HISTORY_SIZE;
        recentDescentMissCount = Math.min(recentDescentMissCount + 1, RECENT_MISS_HISTORY_SIZE);
    }

    private double[] buildRecentMissHistory() {
        int count = Math.min(recentDescentMissCount, RECENT_MISS_HISTORY_SIZE);
        double[] history = new double[count];
        int startIndex =
                (recentDescentMissInsertIndex - count + RECENT_MISS_HISTORY_SIZE) % RECENT_MISS_HISTORY_SIZE;
        for (int i = 0; i < count; i++) {
            history[i] = recentDescentMissMeters[(startIndex + i) % RECENT_MISS_HISTORY_SIZE];
        }
        return history;
    }

    private void publish() {
        int activeCount = 0;
        for (int i = 0; i < FUEL_COUNT; i++) {
            FuelState fuel = fuelStates[i];
            if (fuel.active) {
                activeCount++;
                publishedPoses[i] = new Pose3d(fuel.xMeters, fuel.yMeters, fuel.zMeters, new Rotation3d());
            } else {
                publishedPoses[i] = new Pose3d();
            }
        }

        Logger.recordOutput("Sim/Fuel/poses", publishedPoses);
        Logger.recordOutput("Sim/Fuel/activeCount", activeCount);
        Logger.recordOutput("Sim/Fuel/diameterMeters", FUEL_DIAMETER_METERS);
        Logger.recordOutput("Sim/Fuel/accuracy/descentHeightMeters", DESCENT_ACCURACY_HEIGHT_METERS);
        Logger.recordOutput("Sim/Fuel/accuracy/idealMissMeters", IDEAL_DESCENT_MISS_DISTANCE_METERS);
        Logger.recordOutput("Sim/Fuel/accuracy/latestDescentMissMeters", latestDescentMissMeters);
        Logger.recordOutput("Sim/Fuel/accuracy/latestDescentMissInches", Units.metersToInches(latestDescentMissMeters));
        Logger.recordOutput("Sim/Fuel/accuracy/totalDescentSamples", totalDescentCrossingSamples);
        Logger.recordOutput("Sim/Fuel/accuracy/withinIdealSamples", descentCrossingSamplesWithinIdeal);
        Logger.recordOutput(
                "Sim/Fuel/accuracy/withinIdealFraction",
                totalDescentCrossingSamples > 0
                        ? (double) descentCrossingSamplesWithinIdeal / totalDescentCrossingSamples
                        : 0.0);
        Logger.recordOutput(
                "Sim/Fuel/accuracy/meanDescentMissMeters",
                totalDescentCrossingSamples > 0
                        ? sumDescentMissMeters / totalDescentCrossingSamples
                        : Double.NaN);
        Logger.recordOutput("Sim/Fuel/accuracy/minDescentMissMeters", minDescentMissMeters);
        Logger.recordOutput("Sim/Fuel/accuracy/maxDescentMissMeters", maxDescentMissMeters);
        Logger.recordOutput("Sim/Fuel/accuracy/recentDescentMissMeters", buildRecentMissHistory());
    }

    private record RobotVelocity(double vxMetersPerSec, double vyMetersPerSec, double omegaRadPerSec) {}

    private static final class FuelState {
        double xMeters = 0.0;
        double yMeters = 0.0;
        double zMeters = 0.0;
        double vxMetersPerSec = 0.0;
        double vyMetersPerSec = 0.0;
        double vzMetersPerSec = 0.0;
        boolean active = false;
    }
}
