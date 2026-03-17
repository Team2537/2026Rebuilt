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
 *
 * <p>Uses a profiled heading setpoint, proportional heading feedback, measured-yaw-rate damping,
 * mild setpoint-velocity feedforward, a short target-loss hold window, and slew-limited output.
 */
public class HubAlignController {
    private static final double KP = 6.0;
    private static final double YAW_RATE_DAMPING_GAIN = 0.85;
    private static final double TOLERANCE_RAD = Units.degreesToRadians(1.5);
    private static final double LOW_ERROR_FEEDBACK_ARM_RAD = Units.degreesToRadians(2.0);
    private static final double LOW_ERROR_FEEDBACK_FADE_END_RAD = Units.degreesToRadians(6.0);
    private static final double LOW_ERROR_FEEDBACK_DYNAMIC_TARGET_MAX_RAD_PER_SEC = Units.degreesToRadians(20.0);
    private static final double FEEDFORWARD_FADE_START_RAD = Units.degreesToRadians(1.0);
    private static final double FEEDFORWARD_FADE_END_RAD = Units.degreesToRadians(10.0);
    private static final double PROFILED_SETTLED_VELOCITY_RAD_PER_SEC = Units.degreesToRadians(12.0);
    private static final double MIN_OMEGA_RAD_PER_SEC = 0.001;
    private static final double MAX_OMEGA_RAD_PER_SEC = 5.0;
    private static final double MAX_DAMPING_OMEGA_RAD_PER_SEC = 3.2;
    private static final double MAX_FEEDFORWARD_OMEGA_RAD_PER_SEC = 3.0;
    private static final double OMEGA_SLEW_RATE_RAD_PER_SEC_SQ = 16.0;
    private static final double HEADING_FEEDFORWARD_GAIN = 0.9;
    private static final double TARGET_VELOCITY_FEEDFORWARD_GAIN = 0.6;
    private static final double MAX_TARGET_VELOCITY_RAD_PER_SEC = 3.0;
    private static final int TARGET_VELOCITY_FILTER_TAPS = 3;
    private static final double FEEDFORWARD_DEADBAND_RAD_PER_SEC = 0.05;

    private final ProfiledPIDController pidController;
    private final SlewRateLimiter omegaLimiter;
    private final LinearFilter targetVelocityFilter = LinearFilter.movingAverage(TARGET_VELOCITY_FILTER_TAPS);
    private final TargetHoldover<Rotation2d> targetHoldover =
            new TargetHoldover<>(AutoAimHeadingConfig.TARGET_HOLD_SEC, Timer::getFPGATimestamp);

    private Rotation2d previousEffectiveTarget = null;
    private double previousEffectiveTargetTimeSec = Double.NaN;
    private boolean reachedToleranceOnce = false;

    public HubAlignController() {
        pidController = new ProfiledPIDController(
                KP,
                0.0,
                0.0,
                AutoAimHeadingConfig.createHeadingProfileConstraints());
        pidController.enableContinuousInput(-Math.PI, Math.PI);
        pidController.setTolerance(TOLERANCE_RAD);
        omegaLimiter = new SlewRateLimiter(OMEGA_SLEW_RATE_RAD_PER_SEC_SQ);
    }

    /**
     * Reset the controller state for a new command start.
     *
     * @param currentHeadingRad the robot's current heading in radians
     * @param initialTarget the initial target heading, or null if unavailable
     */
    public void reset(double currentHeadingRad, Rotation2d initialTarget) {
        if (initialTarget != null) {
            targetHoldover.apply(initialTarget);
        } else {
            clearTargetTracking();
        }
        omegaLimiter.reset(0.0);
        pidController.reset(currentHeadingRad, 0.0);
        resetTargetMotionTracking();
        reachedToleranceOnce = false;
    }

    /**
     * Backwards-compatible overload used by tests that do not model measured yaw rate.
     */
    public double calculate(double currentHeadingRad, Rotation2d targetHeading, double fallbackOmega) {
        return calculate(currentHeadingRad, 0.0, targetHeading, fallbackOmega);
    }

    /**
     * Calculate the angular velocity command for heading alignment.
     *
     * <p>When {@code targetHeading} is unavailable, holds the most recent target briefly before
     * falling back to driver omega input.
     *
     * @param currentHeadingRad the robot's current heading in radians
     * @param currentYawRateRadPerSec measured robot yaw rate in rad/s
     * @param targetHeading the desired heading, or null if no target is available
     * @param fallbackOmega omega to return when no target is available (rad/s)
     * @return angular velocity command in rad/s
     */
    public double calculate(
            double currentHeadingRad,
            double currentYawRateRadPerSec,
            Rotation2d targetHeading,
            double fallbackOmega) {
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

        Logger.recordOutput(
                "Drive/AutoAlign/RawTargetDeg",
                targetHeading != null ? targetHeading.getDegrees() : Double.NaN);
        Logger.recordOutput(
                "Drive/AutoAlign/EffectiveTargetDeg",
                effectiveTarget != null ? effectiveTarget.getDegrees() : Double.NaN);
        Logger.recordOutput("Drive/AutoAlign/TargetHeld", targetHeld);
        Logger.recordOutput("Drive/AutoAlign/UsingFallback", effectiveTarget == null);
        Logger.recordOutput("Drive/AutoAlign/TargetAgeSec", targetAgeSec);
        Logger.recordOutput("Drive/AutoAlign/MeasuredOmegaRadPerSec", currentYawRateRadPerSec);

        if (effectiveTarget == null) {
            clearTargetTracking();
            pidController.reset(currentHeadingRad, 0.0);
            resetTargetMotionTracking();
            reachedToleranceOnce = false;
            double fallbackLimited = omegaLimiter.calculate(fallbackOmega);
            Logger.recordOutput("Drive/AutoAlign/HeadingErrorDeg", Double.NaN);
            Logger.recordOutput("Drive/AutoAlign/ProfileSetpointDeg", Double.NaN);
            Logger.recordOutput("Drive/AutoAlign/ProfileVelocityDegPerSec", Double.NaN);
            Logger.recordOutput("Drive/AutoAlign/TargetVelocityDegPerSec", Double.NaN);
            Logger.recordOutput("Drive/AutoAlign/RawFeedbackOmegaRadPerSec", Double.NaN);
            Logger.recordOutput("Drive/AutoAlign/FeedbackOmegaRadPerSec", Double.NaN);
            Logger.recordOutput("Drive/AutoAlign/LowErrorFeedbackScale", Double.NaN);
            Logger.recordOutput("Drive/AutoAlign/FeedforwardScale", Double.NaN);
            Logger.recordOutput("Drive/AutoAlign/FeedforwardOmegaRadPerSec", 0.0);
            Logger.recordOutput("Drive/AutoAlign/DampingOmegaRadPerSec", Double.NaN);
            Logger.recordOutput("Drive/AutoAlign/UnclampedOmegaRadPerSec", fallbackOmega);
            Logger.recordOutput("Drive/AutoAlign/OmegaCommandRadPerSec", fallbackLimited);
            return fallbackLimited;
        }

        double targetHeadingRad = effectiveTarget.getRadians();
        pidController.calculate(currentHeadingRad, targetHeadingRad);
        double profileSetpointRad = pidController.getSetpoint().position;
        double profileVelocityRadPerSec = pidController.getSetpoint().velocity;

        double targetVelocityRadPerSec = 0.0;
        if (previousEffectiveTarget != null && Double.isFinite(previousEffectiveTargetTimeSec)) {
            double dtSec = nowSec - previousEffectiveTargetTimeSec;
            if (dtSec > 1e-6 && dtSec < 0.1) {
                double rawTargetVelocityRadPerSec = MathUtil.angleModulus(
                        effectiveTarget.minus(previousEffectiveTarget).getRadians()) / dtSec;
                targetVelocityRadPerSec = MathUtil.clamp(
                        targetVelocityFilter.calculate(rawTargetVelocityRadPerSec),
                        -MAX_TARGET_VELOCITY_RAD_PER_SEC,
                        MAX_TARGET_VELOCITY_RAD_PER_SEC);
            }
        }
        previousEffectiveTarget = effectiveTarget;
        previousEffectiveTargetTimeSec = nowSec;

        double headingErrorRad = MathUtil.angleModulus(targetHeadingRad - currentHeadingRad);
        double rawFeedbackOmega = KP * headingErrorRad;
        double absHeadingErrorRad = Math.abs(headingErrorRad);

        if (absHeadingErrorRad <= LOW_ERROR_FEEDBACK_ARM_RAD) {
            reachedToleranceOnce = true;
        }

        double feedbackScale = 1.0;
        double absTargetMotionRadPerSec = Math.max(Math.abs(profileVelocityRadPerSec), Math.abs(targetVelocityRadPerSec));
        if (reachedToleranceOnce
                && absTargetMotionRadPerSec <= LOW_ERROR_FEEDBACK_DYNAMIC_TARGET_MAX_RAD_PER_SEC
                && absHeadingErrorRad < LOW_ERROR_FEEDBACK_FADE_END_RAD) {
            feedbackScale = MathUtil.clamp(
                    (absHeadingErrorRad - TOLERANCE_RAD) / (LOW_ERROR_FEEDBACK_FADE_END_RAD - TOLERANCE_RAD),
                    0.0,
                    1.0);
            feedbackScale *= feedbackScale;
        }
        double feedbackOmega = rawFeedbackOmega * feedbackScale;

        double feedforwardScale = MathUtil.clamp(
                (absHeadingErrorRad - FEEDFORWARD_FADE_START_RAD)
                        / (FEEDFORWARD_FADE_END_RAD - FEEDFORWARD_FADE_START_RAD),
                0.0,
                1.0);
        feedforwardScale = 0.15 + 0.85 * feedforwardScale * feedforwardScale;
        double feedforwardOmega = MathUtil.clamp(
                (profileVelocityRadPerSec * HEADING_FEEDFORWARD_GAIN
                                + targetVelocityRadPerSec * TARGET_VELOCITY_FEEDFORWARD_GAIN)
                        * feedforwardScale,
                -MAX_FEEDFORWARD_OMEGA_RAD_PER_SEC,
                MAX_FEEDFORWARD_OMEGA_RAD_PER_SEC);
        if (Math.abs(feedforwardOmega) < FEEDFORWARD_DEADBAND_RAD_PER_SEC) {
            feedforwardOmega = 0.0;
        }

        double dampingOmega = MathUtil.clamp(
                -currentYawRateRadPerSec * YAW_RATE_DAMPING_GAIN,
                -MAX_DAMPING_OMEGA_RAD_PER_SEC,
                MAX_DAMPING_OMEGA_RAD_PER_SEC);

        double unclampedOmega = feedbackOmega + feedforwardOmega + dampingOmega;
        if (absHeadingErrorRad <= TOLERANCE_RAD
                && Math.abs(profileVelocityRadPerSec) <= PROFILED_SETTLED_VELOCITY_RAD_PER_SEC) {
            unclampedOmega = dampingOmega;
        }

        double commandedOmega = MathUtil.clamp(unclampedOmega, -MAX_OMEGA_RAD_PER_SEC, MAX_OMEGA_RAD_PER_SEC);
        if (absHeadingErrorRad > TOLERANCE_RAD
                && Math.abs(commandedOmega) > 0.0
                && Math.abs(commandedOmega) < MIN_OMEGA_RAD_PER_SEC) {
            commandedOmega = Math.copySign(MIN_OMEGA_RAD_PER_SEC, commandedOmega);
        }
        double limitedOmega = omegaLimiter.calculate(commandedOmega);

        Logger.recordOutput("Drive/AutoAlign/HeadingErrorDeg", Units.radiansToDegrees(headingErrorRad));
        Logger.recordOutput("Drive/AutoAlign/ProfileSetpointDeg", Units.radiansToDegrees(profileSetpointRad));
        Logger.recordOutput(
                "Drive/AutoAlign/ProfileVelocityDegPerSec",
                Units.radiansToDegrees(profileVelocityRadPerSec));
        Logger.recordOutput(
                "Drive/AutoAlign/TargetVelocityDegPerSec",
                Units.radiansToDegrees(targetVelocityRadPerSec));
        Logger.recordOutput("Drive/AutoAlign/RawFeedbackOmegaRadPerSec", rawFeedbackOmega);
        Logger.recordOutput("Drive/AutoAlign/FeedbackOmegaRadPerSec", feedbackOmega);
        Logger.recordOutput("Drive/AutoAlign/LowErrorFeedbackScale", feedbackScale);
        Logger.recordOutput("Drive/AutoAlign/FeedforwardScale", feedforwardScale);
        Logger.recordOutput("Drive/AutoAlign/FeedforwardOmegaRadPerSec", feedforwardOmega);
        Logger.recordOutput("Drive/AutoAlign/DampingOmegaRadPerSec", dampingOmega);
        Logger.recordOutput("Drive/AutoAlign/UnclampedOmegaRadPerSec", unclampedOmega);
        Logger.recordOutput("Drive/AutoAlign/OmegaCommandRadPerSec", limitedOmega);
        return limitedOmega;
    }

    private void clearTargetTracking() {
        targetHoldover.clear();
    }

    private void resetTargetMotionTracking() {
        targetVelocityFilter.reset();
        previousEffectiveTarget = null;
        previousEffectiveTargetTimeSec = Double.NaN;
    }
}
