package frc.robot.coordination.shooting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.AimReadyLatch;
import org.littletonrobotics.junction.Logger;

/** Hysteretic heading readiness tracker driven directly from {@link ShotSolution}. */
public final class ShotAimReadiness {
    private AimReadyLatch latch = new AimReadyLatch(Math.toRadians(2.0), Math.toRadians(3.0));
    private double lastEngageToleranceRad = Double.NaN;
    private double lastReleaseToleranceRad = Double.NaN;
    private ShotIntent lastIntent = ShotIntent.NONE;

    public boolean update(ShotSolution solution, Rotation2d robotHeading) {
        if (solution == null || !solution.valid() || solution.desiredRobotHeading() == null || robotHeading == null) {
            reset();
            Logger.recordOutput("Shooting/AimTargetAvailable", false);
            Logger.recordOutput("Shooting/AimErrorRad", Double.NaN);
            Logger.recordOutput("Shooting/AimErrorDeg", Double.NaN);
            Logger.recordOutput("Shooting/AimReadyLatched", false);
            return false;
        }

        if (solution.intent() != lastIntent
                || solution.headingToleranceRad() != lastEngageToleranceRad
                || solution.headingReleaseToleranceRad() != lastReleaseToleranceRad) {
            latch = new AimReadyLatch(solution.headingToleranceRad(), solution.headingReleaseToleranceRad());
            lastIntent = solution.intent();
            lastEngageToleranceRad = solution.headingToleranceRad();
            lastReleaseToleranceRad = solution.headingReleaseToleranceRad();
        }

        double headingErrorRad = MathUtil.angleModulus(solution.desiredRobotHeading().minus(robotHeading).getRadians());
        boolean aimReady = latch.update(Math.abs(headingErrorRad));

        Logger.recordOutput("Shooting/AimTargetAvailable", true);
        Logger.recordOutput(
                "Shooting/TargetHeadingDeg",
                solution.targetHeading() != null ? solution.targetHeading().getDegrees() : Double.NaN);
        Logger.recordOutput("Shooting/DesiredRobotHeadingDeg", solution.desiredRobotHeading().getDegrees());
        Logger.recordOutput("Shooting/AimErrorRad", headingErrorRad);
        Logger.recordOutput("Shooting/AimErrorDeg", Math.toDegrees(headingErrorRad));
        Logger.recordOutput("Shooting/AimReadinessMode", solution.intent().name());
        Logger.recordOutput("Shooting/ActiveAimToleranceRad", solution.headingToleranceRad());
        Logger.recordOutput("Shooting/ActiveAimReleaseToleranceRad", solution.headingReleaseToleranceRad());
        Logger.recordOutput("Shooting/AimReadyLatched", aimReady);
        return aimReady;
    }

    public void reset() {
        latch.reset();
        lastIntent = ShotIntent.NONE;
        lastEngageToleranceRad = Double.NaN;
        lastReleaseToleranceRad = Double.NaN;
    }
}
