package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

/**
 * Heading alignment controller for tracking a dynamic target heading.
 * Uses a profiled PID with a short target-loss hold window and slew-limited output.
 */
public class HubAlignController {
    private static final double KP = 7.0;
    private static final double KD = 0.15;
    private static final double MAX_VELOCITY = 9.0;
    private static final double MAX_ACCELERATION = 36.0;
    private static final double TOLERANCE_RAD = Units.degreesToRadians(0.5);
    private static final double MIN_OMEGA_RAD_PER_SEC = 0.05;
    private static final double MAX_OMEGA_RAD_PER_SEC = 8.0;
    private static final double OMEGA_SLEW_RATE_RAD_PER_SEC_SQ = 45.0;
    private static final double TARGET_HOLD_SEC = 0.12;
    private static final double HEADING_FEEDFORWARD_GAIN = 0.80;
    private static final int TARGET_VELOCITY_FILTER_TAPS = 7;

    private final ProfiledPIDController pidController;
    private final SlewRateLimiter omegaLimiter;
    private final LinearFilter targetVelocityFilter = LinearFilter.movingAverage(TARGET_VELOCITY_FILTER_TAPS);

    private Rotation2d lastTargetHeading = null;
    private double lastTargetTimestampSec = Double.NaN;
    private Rotation2d prevEffectiveTarget = null;
    private double prevEffectiveTargetTimeSec = Double.NaN;

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
            lastTargetHeading = initialTarget;
            lastTargetTimestampSec = Timer.getFPGATimestamp();
        } else {
            clearTargetTracking();
        }
        omegaLimiter.reset(0.0);
        pidController.reset(currentHeadingRad, 0.0);
        targetVelocityFilter.reset();
        prevEffectiveTarget = null;
        prevEffectiveTargetTimeSec = Double.NaN;
    }

    /**
     * Calculate the angular velocity command for heading alignment.
     *
     * <p>When {@code targetHeading} is unavailable, holds the most recent target briefly
     * before falling back to driver omega input.
     *
     * @param currentHeadingRad the robot's current heading in radians
     * @param targetHeading     the desired heading, or null if no target is available
     * @param fallbackOmega     omega to return when no target is available (rad/s)
     * @return angular velocity command in rad/s
     */
    public double calculate(double currentHeadingRad, Rotation2d targetHeading, double fallbackOmega) {
        double nowSec = Timer.getFPGATimestamp();
        if (targetHeading != null) {
            lastTargetHeading = targetHeading;
            lastTargetTimestampSec = nowSec;
        }

        boolean targetHeld = false;
        Rotation2d effectiveTarget = targetHeading;
        if (effectiveTarget == null && lastTargetHeading != null && Double.isFinite(lastTargetTimestampSec)) {
            double targetAgeSec = nowSec - lastTargetTimestampSec;
            if (targetAgeSec <= TARGET_HOLD_SEC) {
                effectiveTarget = lastTargetHeading;
                targetHeld = true;
            }
        }

        Logger.recordOutput("Drive/AutoAlign/RawTargetDeg",
                targetHeading != null ? targetHeading.getDegrees() : Double.NaN);
        Logger.recordOutput("Drive/AutoAlign/EffectiveTargetDeg",
                effectiveTarget != null ? effectiveTarget.getDegrees() : Double.NaN);
        Logger.recordOutput("Drive/AutoAlign/TargetHeld", targetHeld);
        Logger.recordOutput("Drive/AutoAlign/UsingFallback", effectiveTarget == null);
        Logger.recordOutput(
                "Drive/AutoAlign/TargetAgeSec",
                Double.isFinite(lastTargetTimestampSec) ? nowSec - lastTargetTimestampSec : Double.NaN);

        // Compute target heading velocity for feedforward — helps track a moving aim point
        double targetVelocityRadPerSec = 0.0;
        if (effectiveTarget != null && prevEffectiveTarget != null
                && Double.isFinite(prevEffectiveTargetTimeSec)) {
            double dt = nowSec - prevEffectiveTargetTimeSec;
            if (dt > 1e-6 && dt < 0.1) {
                double rawVelocity = MathUtil.angleModulus(
                        effectiveTarget.minus(prevEffectiveTarget).getRadians()) / dt;
                targetVelocityRadPerSec = targetVelocityFilter.calculate(rawVelocity);
            }
        }
        if (effectiveTarget != null) {
            prevEffectiveTarget = effectiveTarget;
            prevEffectiveTargetTimeSec = nowSec;
        } else {
            prevEffectiveTarget = null;
            prevEffectiveTargetTimeSec = Double.NaN;
            targetVelocityFilter.reset();
        }
        double feedforwardOmega = targetVelocityRadPerSec * HEADING_FEEDFORWARD_GAIN;

        if (effectiveTarget == null) {
            clearTargetTracking();
            pidController.reset(currentHeadingRad, 0.0);
            double fallbackLimited = omegaLimiter.calculate(fallbackOmega);
            Logger.recordOutput("Drive/AutoAlign/HeadingErrorDeg", Double.NaN);
            Logger.recordOutput("Drive/AutoAlign/FeedforwardOmegaRadPerSec", 0.0);
            Logger.recordOutput("Drive/AutoAlign/OmegaCommandRadPerSec", fallbackLimited);
            return fallbackLimited;
        }

        double targetHeadingRad = effectiveTarget.getRadians();
        double headingErrorRad = MathUtil.angleModulus(targetHeadingRad - currentHeadingRad);
        double feedbackOmega = pidController.calculate(currentHeadingRad, targetHeadingRad);
        double commandedOmega;
        if (Math.abs(headingErrorRad) <= TOLERANCE_RAD) {
            // Within tolerance: use only feedforward to maintain smooth tracking of a
            // moving target. Without this, the robot would stop, drift out of tolerance,
            // correct, and stop again — causing chatter.
            commandedOmega = feedforwardOmega;
        } else {
            commandedOmega = MathUtil.clamp(
                    feedbackOmega + feedforwardOmega, -MAX_OMEGA_RAD_PER_SEC, MAX_OMEGA_RAD_PER_SEC);
        }
        if (Math.abs(commandedOmega) > 0.0 && Math.abs(commandedOmega) < MIN_OMEGA_RAD_PER_SEC) {
            commandedOmega = Math.copySign(MIN_OMEGA_RAD_PER_SEC, commandedOmega);
        }
        double limitedOmega = omegaLimiter.calculate(commandedOmega);

        Logger.recordOutput("Drive/AutoAlign/HeadingErrorDeg", Units.radiansToDegrees(headingErrorRad));
        Logger.recordOutput("Drive/AutoAlign/FeedforwardOmegaRadPerSec", feedforwardOmega);
        Logger.recordOutput("Drive/AutoAlign/OmegaCommandRadPerSec", limitedOmega);
        return limitedOmega;
    }

    private void clearTargetTracking() {
        lastTargetHeading = null;
        lastTargetTimestampSec = Double.NaN;
    }
}
