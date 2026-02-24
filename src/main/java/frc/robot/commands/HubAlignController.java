package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

/**
 * Heading alignment controller for tracking a dynamic target heading (e.g., hub-facing).
 * Encapsulates filtered target tracking, derivative-based feedforward, hold-at-target logic,
 * and profiled PID control with slew-rate-limited output.
 */
public class HubAlignController {
    private static final double KP = 2.4;
    private static final double KD = 0.16;
    private static final double MAX_VELOCITY = 5.2;
    private static final double MAX_ACCELERATION = 14.0;
    private static final double TOLERANCE_RAD = Units.degreesToRadians(1.25);
    private static final double HOLD_RELEASE_TOLERANCE_RAD = Units.degreesToRadians(1.7);
    private static final double TARGET_FILTER_ALPHA = 0.2;
    private static final double TARGET_FILTER_MAX_STEP_RAD = Units.degreesToRadians(4.0);
    private static final double MAX_OMEGA_RAD_PER_SEC = 4.5;
    private static final double OMEGA_SLEW_RATE_RAD_PER_SEC_SQ = 36.0;
    private static final double TARGET_RATE_FILTER_ALPHA = 0.35;
    private static final double TARGET_RATE_MAX_RAD_PER_SEC = 6.0;
    private static final double TARGET_RATE_FF = 0.9;
    private static final double MOVING_TARGET_RATE_THRESHOLD_RAD_PER_SEC = 0.35;
    private static final double TARGET_RATE_DT_MIN_SEC = 1e-3;
    private static final double TARGET_RATE_DT_MAX_SEC = 0.1;

    private final ProfiledPIDController pidController;
    private final SlewRateLimiter omegaLimiter;

    private Rotation2d filteredTargetHeading = null;
    private boolean holdAtTarget = false;
    private Rotation2d lastFilteredTargetHeading = null;
    private double lastFilteredTargetTimestampSec = Double.NaN;
    private double filteredTargetRateRadPerSec = 0.0;

    public HubAlignController() {
        pidController = new ProfiledPIDController(
                KP, 0.0, KD,
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
        pidController.enableContinuousInput(-Math.PI, Math.PI);
        pidController.setTolerance(TOLERANCE_RAD);
        omegaLimiter = new SlewRateLimiter(OMEGA_SLEW_RATE_RAD_PER_SEC_SQ);
    }

    /**
     * Reset the controller state for a new command start.
     *
     * @param currentHeadingRad the robot's current heading in radians
     * @param initialTarget     the initial target heading, or null if unavailable
     */
    public void reset(double currentHeadingRad, Rotation2d initialTarget) {
        if (initialTarget != null) {
            initializeTargetTracking(initialTarget, Timer.getFPGATimestamp());
        } else {
            clearTargetTracking();
        }
        omegaLimiter.reset(0.0);
        pidController.reset(currentHeadingRad);
    }

    /**
     * Calculate the angular velocity command for heading alignment.
     *
     * <p>When {@code targetHeading} is non-null, returns a PID-controlled omega with
     * derivative feedforward and slew-rate limiting. When {@code targetHeading} is null,
     * resets internal tracking state and returns {@code fallbackOmega} unchanged.
     *
     * @param currentHeadingRad the robot's current heading in radians
     * @param targetHeading     the desired heading, or null if no target is available
     * @param fallbackOmega     omega to return when no target is available (rad/s)
     * @return angular velocity command in rad/s
     */
    public double calculate(double currentHeadingRad, Rotation2d targetHeading, double fallbackOmega) {
        if (targetHeading == null) {
            clearTargetTracking();
            omegaLimiter.reset(fallbackOmega);
            pidController.reset(currentHeadingRad);
            return fallbackOmega;
        }

        double nowSec = Timer.getFPGATimestamp();
        if (filteredTargetHeading == null) {
            initializeTargetTracking(targetHeading, nowSec);
            pidController.reset(currentHeadingRad);
        } else {
            filteredTargetHeading = filterTargetHeading(filteredTargetHeading, targetHeading);
        }

        double targetRateRadPerSec = updateTargetRate(nowSec);
        double filteredTargetRad = filteredTargetHeading.getRadians();
        double headingErrorRad = MathUtil.angleModulus(filteredTargetRad - currentHeadingRad);
        boolean targetMoving =
                Math.abs(targetRateRadPerSec) >= MOVING_TARGET_RATE_THRESHOLD_RAD_PER_SEC;

        double omega;
        if (holdAtTarget && !targetMoving
                && Math.abs(headingErrorRad) <= HOLD_RELEASE_TOLERANCE_RAD) {
            omega = 0.0;
            pidController.reset(currentHeadingRad);
        } else if (!targetMoving && Math.abs(headingErrorRad) <= TOLERANCE_RAD) {
            holdAtTarget = true;
            omega = 0.0;
            pidController.reset(currentHeadingRad);
        } else {
            holdAtTarget = false;
            double feedbackOmega = pidController.calculate(currentHeadingRad, filteredTargetRad);
            double feedforwardOmega = targetRateRadPerSec * TARGET_RATE_FF;
            omega = MathUtil.clamp(
                    feedbackOmega + feedforwardOmega,
                    -MAX_OMEGA_RAD_PER_SEC,
                    MAX_OMEGA_RAD_PER_SEC);
        }

        return omegaLimiter.calculate(omega);
    }

    private void initializeTargetTracking(Rotation2d initialHeading, double nowSec) {
        filteredTargetHeading = initialHeading;
        holdAtTarget = false;
        lastFilteredTargetHeading = initialHeading;
        lastFilteredTargetTimestampSec = nowSec;
        filteredTargetRateRadPerSec = 0.0;
    }

    private void clearTargetTracking() {
        filteredTargetHeading = null;
        holdAtTarget = false;
        lastFilteredTargetHeading = null;
        lastFilteredTargetTimestampSec = Double.NaN;
        filteredTargetRateRadPerSec = 0.0;
    }

    private static Rotation2d filterTargetHeading(Rotation2d filtered, Rotation2d raw) {
        double delta = MathUtil.angleModulus(raw.getRadians() - filtered.getRadians());
        double step = MathUtil.clamp(
                delta * TARGET_FILTER_ALPHA,
                -TARGET_FILTER_MAX_STEP_RAD,
                TARGET_FILTER_MAX_STEP_RAD);
        return new Rotation2d(filtered.getRadians() + step);
    }

    private double updateTargetRate(double nowSec) {
        if (lastFilteredTargetHeading == null
                || !Double.isFinite(lastFilteredTargetTimestampSec)) {
            lastFilteredTargetHeading = filteredTargetHeading;
            lastFilteredTargetTimestampSec = nowSec;
            filteredTargetRateRadPerSec = 0.0;
            return 0.0;
        }

        double dtSec = MathUtil.clamp(
                nowSec - lastFilteredTargetTimestampSec,
                TARGET_RATE_DT_MIN_SEC,
                TARGET_RATE_DT_MAX_SEC);
        double headingDelta = MathUtil.angleModulus(
                filteredTargetHeading.getRadians()
                        - lastFilteredTargetHeading.getRadians());
        double rawRate = MathUtil.clamp(
                headingDelta / dtSec,
                -TARGET_RATE_MAX_RAD_PER_SEC,
                TARGET_RATE_MAX_RAD_PER_SEC);
        filteredTargetRateRadPerSec +=
                TARGET_RATE_FILTER_ALPHA * (rawRate - filteredTargetRateRadPerSec);
        lastFilteredTargetHeading = filteredTargetHeading;
        lastFilteredTargetTimestampSec = nowSec;
        return filteredTargetRateRadPerSec;
    }
}
