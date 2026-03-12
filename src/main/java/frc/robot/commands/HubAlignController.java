package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.AutoAimHeadingConfig;
import frc.robot.util.TargetHoldover;
import org.littletonrobotics.junction.Logger;

/**
 * Heading alignment controller for tracking a dynamic target heading.
 * Uses a profiled PID with a short target-loss hold window and slew-limited output.
 */
public class HubAlignController {
    private static final double KP = 6.5;
    private static final double KD = 0.3;
    private static final double TOLERANCE_RAD = Units.degreesToRadians(2.0);
    private static final double LOW_ERROR_FEEDBACK_ARM_RAD = Units.degreesToRadians(1.0);
    private static final double LOW_ERROR_FEEDBACK_FADE_END_RAD = Units.degreesToRadians(4.0);
    private static final double LOW_ERROR_FEEDBACK_STATIC_TARGET_MAX_RAD_PER_SEC = Units.degreesToRadians(8.0);
    private static final double MIN_OMEGA_RAD_PER_SEC = 0.001;
    private static final double MAX_OMEGA_RAD_PER_SEC = 5.0;
    private static final double OMEGA_SLEW_RATE_RAD_PER_SEC_SQ = 18.0;
    private static final double HEADING_FEEDFORWARD_GAIN = 0.6;
    private static final double MAX_TARGET_VELOCITY_RAD_PER_SEC = 5.0;
    private static final double FEEDFORWARD_DEADBAND_RAD_PER_SEC = 0.02;
    private static final int TARGET_VELOCITY_FILTER_TAPS = 9;

    private final ProfiledPIDController pidController;
    private final SlewRateLimiter omegaLimiter;
    private final LinearFilter targetVelocityFilter = LinearFilter.movingAverage(TARGET_VELOCITY_FILTER_TAPS);
    private final TargetHoldover<Rotation2d> targetHoldover =
            new TargetHoldover<>(AutoAimHeadingConfig.TARGET_HOLD_SEC, Timer::getFPGATimestamp);

    private Rotation2d prevEffectiveTarget = null;
    private double prevEffectiveTargetTimeSec = Double.NaN;
    private boolean reachedToleranceOnce = false;

    public HubAlignController() {
        pidController = new ProfiledPIDController(
                KP, 0.0, KD,
                AutoAimHeadingConfig.createHeadingProfileConstraints());
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
            targetHoldover.apply(initialTarget);
        } else {
            clearTargetTracking();
        }
        omegaLimiter.reset(0.0);
        pidController.reset(currentHeadingRad, 0.0);
        targetVelocityFilter.reset();
        prevEffectiveTarget = null;
        prevEffectiveTargetTimeSec = Double.NaN;
        reachedToleranceOnce = false;
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

        Rotation2d effectiveTarget = targetHeading;
        boolean targetHeld = false;
        double targetAgeSec = Double.NaN;
        boolean allowTargetHold = Math.abs(fallbackOmega) > 1e-4;
        if (allowTargetHold) {
            TargetHoldover.HoldResult<Rotation2d> heldTarget = targetHoldover.apply(targetHeading);
            effectiveTarget = heldTarget.value();
            targetHeld = heldTarget.held();
            targetAgeSec = heldTarget.ageSec();
        } else if (targetHeading != null) {
            targetHoldover.apply(targetHeading);
        } else {
            targetHoldover.clear();
        }

        Logger.recordOutput("Drive/AutoAlign/RawTargetDeg",
                targetHeading != null ? targetHeading.getDegrees() : Double.NaN);
        Logger.recordOutput("Drive/AutoAlign/EffectiveTargetDeg",
                effectiveTarget != null ? effectiveTarget.getDegrees() : Double.NaN);
        Logger.recordOutput("Drive/AutoAlign/TargetHeld", targetHeld);
        Logger.recordOutput("Drive/AutoAlign/UsingFallback", effectiveTarget == null);
        Logger.recordOutput(
                "Drive/AutoAlign/TargetAgeSec",
                targetAgeSec);

        // Compute target heading velocity for feedforward — helps track a moving aim point
        double targetVelocityRadPerSec = 0.0;
        if (effectiveTarget != null && prevEffectiveTarget != null
                && Double.isFinite(prevEffectiveTargetTimeSec)) {
            double dt = nowSec - prevEffectiveTargetTimeSec;
            if (dt > 1e-6 && dt < 0.1) {
                double rawVelocity = MathUtil.angleModulus(
                        effectiveTarget.minus(prevEffectiveTarget).getRadians()) / dt;
                double filteredVelocity = targetVelocityFilter.calculate(rawVelocity);
                targetVelocityRadPerSec = MathUtil.clamp(
                        filteredVelocity,
                        -MAX_TARGET_VELOCITY_RAD_PER_SEC,
                        MAX_TARGET_VELOCITY_RAD_PER_SEC);
            }
        }
        if (effectiveTarget != null) {
            prevEffectiveTarget = effectiveTarget;
            prevEffectiveTargetTimeSec = nowSec;
        } else {
            prevEffectiveTarget = null;
            prevEffectiveTargetTimeSec = Double.NaN;
            targetVelocityFilter.reset();
            reachedToleranceOnce = false;
        }
        double feedforwardOmega = MathUtil.clamp(
                targetVelocityRadPerSec * HEADING_FEEDFORWARD_GAIN,
                -MAX_OMEGA_RAD_PER_SEC,
                MAX_OMEGA_RAD_PER_SEC);
        if (Math.abs(feedforwardOmega) < FEEDFORWARD_DEADBAND_RAD_PER_SEC) {
            feedforwardOmega = 0.0;
        }

        if (effectiveTarget == null) {
            clearTargetTracking();
            pidController.reset(currentHeadingRad, 0.0);
            reachedToleranceOnce = false;
            double fallbackLimited = omegaLimiter.calculate(fallbackOmega);
            Logger.recordOutput("Drive/AutoAlign/HeadingErrorDeg", Double.NaN);
            Logger.recordOutput("Drive/AutoAlign/FeedforwardOmegaRadPerSec", 0.0);
            Logger.recordOutput("Drive/AutoAlign/LowErrorFeedbackScale", Double.NaN);
            Logger.recordOutput("Drive/AutoAlign/OmegaCommandRadPerSec", fallbackLimited);
            return fallbackLimited;
        }

        double targetHeadingRad = effectiveTarget.getRadians();
        double headingErrorRad = MathUtil.angleModulus(targetHeadingRad - currentHeadingRad);
        double rawFeedbackOmega = pidController.calculate(currentHeadingRad, targetHeadingRad);
        double absHeadingErrorRad = Math.abs(headingErrorRad);
        double absTargetVelocityRadPerSec = Math.abs(targetVelocityRadPerSec);
        if (absHeadingErrorRad <= LOW_ERROR_FEEDBACK_ARM_RAD) {
            reachedToleranceOnce = true;
        }
        double feedbackScale = 1.0;
        if (reachedToleranceOnce
                && absTargetVelocityRadPerSec <= LOW_ERROR_FEEDBACK_STATIC_TARGET_MAX_RAD_PER_SEC
                && absHeadingErrorRad < LOW_ERROR_FEEDBACK_FADE_END_RAD) {
            feedbackScale = MathUtil.clamp(
                    (absHeadingErrorRad - TOLERANCE_RAD) / (LOW_ERROR_FEEDBACK_FADE_END_RAD - TOLERANCE_RAD),
                    0.0,
                    1.0);
            feedbackScale *= feedbackScale;
        }
        double feedbackOmega = rawFeedbackOmega * feedbackScale;

        double commandedOmega;
        if (absHeadingErrorRad <= TOLERANCE_RAD) {
            // Within tolerance: use only feedforward to maintain smooth tracking of a
            // moving target. Without this, the robot would stop, drift out of tolerance,
            // correct, and stop again — causing chatter.
            commandedOmega = feedforwardOmega;
        } else {
            commandedOmega = MathUtil.clamp(
                    feedbackOmega + feedforwardOmega, -MAX_OMEGA_RAD_PER_SEC, MAX_OMEGA_RAD_PER_SEC);
        }
        if (absHeadingErrorRad > TOLERANCE_RAD
                && Math.abs(commandedOmega) > 0.0
                && Math.abs(commandedOmega) < MIN_OMEGA_RAD_PER_SEC) {
            commandedOmega = Math.copySign(MIN_OMEGA_RAD_PER_SEC, commandedOmega);
        }
        double limitedOmega = omegaLimiter.calculate(commandedOmega);

        Logger.recordOutput("Drive/AutoAlign/HeadingErrorDeg", Units.radiansToDegrees(headingErrorRad));
        Logger.recordOutput("Drive/AutoAlign/FeedforwardOmegaRadPerSec", feedforwardOmega);
        Logger.recordOutput("Drive/AutoAlign/LowErrorFeedbackScale", feedbackScale);
        Logger.recordOutput("Drive/AutoAlign/OmegaCommandRadPerSec", limitedOmega);
        return limitedOmega;
    }

    private void clearTargetTracking() {
        targetHoldover.clear();
    }
}
