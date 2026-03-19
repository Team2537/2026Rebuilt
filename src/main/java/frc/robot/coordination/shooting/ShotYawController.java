package frc.robot.coordination.shooting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Simple shot-yaw controller inspired by MA's launch aiming structure.
 *
 * <p>Commanded omega is target heading-rate feedforward plus heading error P and angular-rate D.
 */
public final class ShotYawController {
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("ShotYaw/kP", 8.0);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("ShotYaw/kD", 0.5);
    private static final LoggedTunableNumber maxFeedbackOmegaRadPerSec =
            new LoggedTunableNumber("ShotYaw/MaxFeedbackOmegaRadPerSec", Math.toRadians(540.0));

    public double calculate(
            Rotation2d currentHeading,
            double measuredOmegaRadPerSec,
            Rotation2d desiredHeading,
            double desiredHeadingRateRadPerSec,
            double maxOmegaRadPerSec) {
        if (currentHeading == null || desiredHeading == null) {
            Logger.recordOutput("ShotYaw/HeadingErrorDeg", Double.NaN);
            Logger.recordOutput("ShotYaw/DesiredHeadingDeg", Double.NaN);
            Logger.recordOutput("ShotYaw/DesiredHeadingRateRadPerSec", 0.0);
            Logger.recordOutput("ShotYaw/MeasuredOmegaRadPerSec", measuredOmegaRadPerSec);
            Logger.recordOutput("ShotYaw/OmegaCommandRadPerSec", 0.0);
            return 0.0;
        }

        double headingErrorRad = MathUtil.angleModulus(desiredHeading.minus(currentHeading).getRadians());
        double feedbackOmega = kP.get() * headingErrorRad + kD.get() * (desiredHeadingRateRadPerSec - measuredOmegaRadPerSec);
        double omegaCommand = desiredHeadingRateRadPerSec + feedbackOmega;
        double limitedOmega = MathUtil.clamp(omegaCommand, -maxOmegaRadPerSec, maxOmegaRadPerSec);

        Logger.recordOutput("ShotYaw/HeadingErrorDeg", Math.toDegrees(headingErrorRad));
        Logger.recordOutput("ShotYaw/DesiredHeadingDeg", desiredHeading.getDegrees());
        Logger.recordOutput("ShotYaw/DesiredHeadingRateRadPerSec", desiredHeadingRateRadPerSec);
        Logger.recordOutput("ShotYaw/MeasuredOmegaRadPerSec", measuredOmegaRadPerSec);
        Logger.recordOutput("ShotYaw/FeedbackOmegaRadPerSec", feedbackOmega);
        Logger.recordOutput("ShotYaw/OmegaCommandRadPerSec", limitedOmega);
        return limitedOmega;
    }

    /**
     * Returns only the corrective rotational feedback term. Use this with PathPlanner's
     * overrideRotationFeedback API so the library can keep owning its feedforward path logic.
     */
    public double calculateFeedback(
            Rotation2d currentHeading,
            double measuredOmegaRadPerSec,
            Rotation2d desiredHeading,
            double desiredHeadingRateRadPerSec,
            double maxOmegaRadPerSec) {
        if (currentHeading == null || desiredHeading == null) {
            return 0.0;
        }
        double headingErrorRad = MathUtil.angleModulus(desiredHeading.minus(currentHeading).getRadians());
        double feedbackOmega = kP.get() * headingErrorRad + kD.get() * (desiredHeadingRateRadPerSec - measuredOmegaRadPerSec);
        double feedbackLimit = Math.min(maxOmegaRadPerSec, maxFeedbackOmegaRadPerSec.get());
        return MathUtil.clamp(feedbackOmega, -feedbackLimit, feedbackLimit);
    }

    public double calculate(
            Rotation2d currentHeading,
            ChassisSpeeds measuredRobotSpeeds,
            Rotation2d desiredHeading,
            double desiredHeadingRateRadPerSec,
            double maxOmegaRadPerSec) {
        return calculate(
                currentHeading,
                measuredRobotSpeeds != null ? measuredRobotSpeeds.omegaRadiansPerSecond : 0.0,
                desiredHeading,
                desiredHeadingRateRadPerSec,
                maxOmegaRadPerSec);
    }
}
